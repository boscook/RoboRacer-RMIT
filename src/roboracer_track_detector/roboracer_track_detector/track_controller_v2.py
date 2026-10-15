#!/usr/bin/env python3

import math
from collections import deque

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import LaserScan
from autoware_control_msgs.msg import Control


class TrackController(Node):

    NORMAL = 0
    AVOID_LEFT = 1
    AVOID_RIGHT = 2
    EMERGENCY_STOP = 3

    def __init__(self):
        super().__init__('roboracer_track_controller')

        # ==================================================
        # SPEED
        # ==================================================

        self.enable_motor = True

        self.drive_velocity = 10.0
        self.approach_velocity = 0.12
        self.avoid_velocity = 0.10

        # ==================================================
        # DISTANCE THRESHOLDS
        # ==================================================

        # Normal obstacle avoidance.
        self.avoid_distance = 0.50

        # If an obstacle is approaching quickly, begin
        # avoidance earlier.
        self.dynamic_avoid_distance = 1.00

        # Absolute emergency stop.
        self.stop_distance = 0.15

        # Return to normal when sufficiently clear.
        self.clear_distance = 0.90

        # ==================================================
        # DYNAMIC OBJECT DETECTION
        # ==================================================

        # Negative range rate = object getting closer.
        #
        # Example:
        # -0.50 m/s means measured obstacle distance
        # is decreasing by ~0.5 m every second.
        self.approaching_threshold = -0.20

        # Positive enough = obstacle moving away.
        self.moving_away_threshold = 0.15

        # Smooth the front-distance history.
        self.distance_history = deque(maxlen=8)

        self.range_rate = 0.0

        # ==================================================
        # STEERING
        # ==================================================

        self.max_steering_angle = 0.35

        self.steering_gain = 1.35

        self.avoid_steering = 0.30

        # ==================================================
        # STATE TIMING
        # ==================================================

        self.target_timeout = 0.5

        # Prevent left/right oscillation.
        self.min_avoid_time = 0.70

        self.state = self.NORMAL

        self.target_angle = 0.0
        self.last_target_time = None

        self.front_distance = float('inf')
        self.left_clearance = float('inf')
        self.right_clearance = float('inf')

        self.avoid_start_time = None

        # ==================================================
        # ROS
        # ==================================================

        self.target_sub = self.create_subscription(
            PointStamped,
            '/roboracer/target_point',
            self.target_callback,
            10
        )

        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.control_pub = self.create_publisher(
            Control,
            '/control/command/control_cmd',
            10
        )

        self.timer = self.create_timer(
            0.05,
            self.control_loop
        )

        self.get_logger().info(
            'Dynamic Obstacle Avoidance V2 started'
        )

    # ==================================================
    # Track target
    # ==================================================

    def target_callback(self, msg):

        x = float(msg.point.x)
        y = float(msg.point.y)

        if x <= 0.05:
            return

        self.target_angle = math.atan2(y, x)
        self.last_target_time = self.get_clock().now()

    # ==================================================
    # Robust LiDAR sector measurement
    # ==================================================

    def sector_clearance(
        self,
        scan,
        angle_min_deg,
        angle_max_deg
    ):

        values = []

        amin = math.radians(angle_min_deg)
        amax = math.radians(angle_max_deg)

        for i, distance in enumerate(scan.ranges):

            angle = (
                scan.angle_min +
                i * scan.angle_increment
            )

            if angle < amin or angle > amax:
                continue

            if not math.isfinite(distance):
                continue

            if distance < scan.range_min:
                continue

            if distance > scan.range_max:
                continue

            values.append(float(distance))

        if not values:
            return float('inf')

        values.sort()

        # 10th percentile rather than one noisy minimum ray.
        index = int(0.10 * (len(values) - 1))

        return values[index]

    # ==================================================
    # LiDAR callback + motion estimation
    # ==================================================

    def scan_callback(self, scan):

        now = self.get_clock().now()

        front = self.sector_clearance(
            scan,
            -10.0,
            10.0
        )

        left = self.sector_clearance(
            scan,
            15.0,
            70.0
        )

        right = self.sector_clearance(
            scan,
            -70.0,
            -15.0
        )

        self.front_distance = front
        self.left_clearance = left
        self.right_clearance = right

        # --------------------------------------------------
        # Estimate front obstacle range rate
        # --------------------------------------------------

        if math.isfinite(front):

            t = now.nanoseconds / 1e9

            self.distance_history.append(
                (t, front)
            )

            # Compare oldest and newest samples.
            if len(self.distance_history) >= 4:

                old_t, old_d = self.distance_history[0]
                new_t, new_d = self.distance_history[-1]

                dt = new_t - old_t

                if dt > 0.05:

                    raw_rate = (
                        new_d - old_d
                    ) / dt

                    # Clamp ridiculous scan jumps.
                    raw_rate = max(
                        -3.0,
                        min(3.0, raw_rate)
                    )

                    # Low-pass filter.
                    self.range_rate = (
                        0.70 * self.range_rate +
                        0.30 * raw_rate
                    )

        else:

            # No valid obstacle directly ahead.
            self.distance_history.clear()
            self.range_rate = 0.0

    # ==================================================
    # State helpers
    # ==================================================

    def state_name(self):

        names = {
            self.NORMAL: 'NORMAL',
            self.AVOID_LEFT: 'AVOID_LEFT',
            self.AVOID_RIGHT: 'AVOID_RIGHT',
            self.EMERGENCY_STOP: 'STOP'
        }

        return names.get(
            self.state,
            'UNKNOWN'
        )

    def set_state(self, new_state):

        if new_state == self.state:
            return

        old_name = self.state_name()

        self.state = new_state

        new_name = self.state_name()

        self.get_logger().warn(
            f'STATE: {old_name} -> {new_name}'
        )

        if new_state in (
            self.AVOID_LEFT,
            self.AVOID_RIGHT
        ):

            self.avoid_start_time = (
                self.get_clock().now()
            )

    def avoidance_time(self):

        if self.avoid_start_time is None:
            return 0.0

        return (
            self.get_clock().now() -
            self.avoid_start_time
        ).nanoseconds / 1e9

    def choose_escape_side(self):

        # Add a small bias requirement so tiny LiDAR
        # differences do not constantly change the choice.
        margin = 0.08

        if (
            self.left_clearance >
            self.right_clearance + margin
        ):

            return self.AVOID_LEFT

        if (
            self.right_clearance >
            self.left_clearance + margin
        ):

            return self.AVOID_RIGHT

        # If almost equal, prefer the current track target.
        if self.target_angle >= 0.0:
            return self.AVOID_LEFT

        return self.AVOID_RIGHT

    # ==================================================
    # Main control loop
    # ==================================================

    def control_loop(self):

        msg = Control()

        # --------------------------------------------------
        # Track target freshness
        # --------------------------------------------------

        target_valid = False

        if self.last_target_time is not None:

            age = (
                self.get_clock().now() -
                self.last_target_time
            ).nanoseconds / 1e9

            target_valid = (
                age <= self.target_timeout
            )

        approaching = (
            self.range_rate <
            self.approaching_threshold
        )

        moving_away = (
            self.range_rate >
            self.moving_away_threshold
        )

        # --------------------------------------------------
        # 1. Absolute emergency stop
        # --------------------------------------------------

        if (
            self.front_distance <
            self.stop_distance
        ):

            self.set_state(
                self.EMERGENCY_STOP
            )

        # --------------------------------------------------
        # 2. Recover from emergency stop
        # --------------------------------------------------

        elif self.state == self.EMERGENCY_STOP:

            if (
                self.front_distance >
                self.stop_distance + 0.12
            ):

                self.set_state(
                    self.choose_escape_side()
                )

        # --------------------------------------------------
        # 3. Normal driving
        # --------------------------------------------------

        elif self.state == self.NORMAL:

            # Dynamic object approaching:
            # react earlier than for a stationary object.
            if (
                approaching and
                self.front_distance <
                self.dynamic_avoid_distance
            ):

                self.set_state(
                    self.choose_escape_side()
                )

            # Ordinary/static obstacle.
            elif (
                self.front_distance <
                self.avoid_distance
            ):

                self.set_state(
                    self.choose_escape_side()
                )

        # --------------------------------------------------
        # 4. Committed avoidance
        # --------------------------------------------------

        elif self.state in (
            self.AVOID_LEFT,
            self.AVOID_RIGHT
        ):

            if (
                self.avoidance_time() >=
                self.min_avoid_time
            ):

                # Return to normal when path is clear.
                if (
                    self.front_distance >
                    self.clear_distance
                ):

                    self.set_state(
                        self.NORMAL
                    )

                # If the object is clearly moving away,
                # allow slightly earlier recovery.
                elif (
                    moving_away and
                    self.front_distance >
                    self.avoid_distance
                ):

                    self.set_state(
                        self.NORMAL
                    )

        # --------------------------------------------------
        # Steering
        # --------------------------------------------------

        if self.state == self.AVOID_LEFT:

            steering = self.avoid_steering

        elif self.state == self.AVOID_RIGHT:

            steering = -self.avoid_steering

        elif target_valid:

            steering = (
                self.steering_gain *
                self.target_angle
            )

        else:

            steering = 0.0

        steering = max(
            -self.max_steering_angle,
            min(
                self.max_steering_angle,
                steering
            )
        )

        # --------------------------------------------------
        # Velocity
        # --------------------------------------------------

        if (
            not self.enable_motor or
            not target_valid
        ):

            velocity = 0.0

        elif self.state == self.EMERGENCY_STOP:

            velocity = 0.0

        elif self.state in (
            self.AVOID_LEFT,
            self.AVOID_RIGHT
        ):

            velocity = self.avoid_velocity

        elif approaching:

            # Object approaching but not yet inside the
            # avoidance trigger.
            velocity = self.approach_velocity

        else:

            velocity = self.drive_velocity

        # --------------------------------------------------
        # Publish Autoware Control
        # --------------------------------------------------

        msg.lateral.steering_tire_angle = (
            steering
        )

        msg.longitudinal.velocity = (
            velocity
        )

        self.control_pub.publish(msg)

        # --------------------------------------------------
        # Diagnostics
        # --------------------------------------------------

        if approaching:
            motion = 'APPROACHING'

        elif moving_away:
            motion = 'MOVING_AWAY'

        else:
            motion = 'STABLE'

        self.get_logger().info(
            f'{self.state_name()} | '
            f'{motion} | '
            f'front={self.front_distance:.2f}m '
            f'rate={self.range_rate:+.2f}m/s | '
            f'L={self.left_clearance:.2f} '
            f'R={self.right_clearance:.2f} | '
            f'v={velocity:.2f} '
            f'steer={math.degrees(steering):+.1f}deg',
            throttle_duration_sec=0.5
        )


def main(args=None):

    rclpy.init(args=args)

    node = TrackController()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    # Explicit stop on shutdown.
    stop = Control()

    stop.longitudinal.velocity = 0.0
    stop.lateral.steering_tire_angle = 0.0

    node.control_pub.publish(stop)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
