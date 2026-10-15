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
    RECOVER = 4

    def __init__(self):
        super().__init__('roboracer_track_controller')

        # ==================================================
        # SPEED
        # ==================================================

        self.enable_motor = True

        self.drive_velocity = 0.20
        self.approach_velocity = 0.12
        self.avoid_velocity = 0.10
        self.recover_velocity = 0.12

        # ==================================================
        # DISTANCES
        # ==================================================

        self.avoid_distance = 0.70
        self.dynamic_avoid_distance = 1.00

        self.stop_distance = 0.25

        # Obstacle considered passed.
        self.clear_distance = 0.90

        # ==================================================
        # DYNAMIC OBJECT DETECTION
        # ==================================================

        self.approaching_threshold = -0.20
        self.moving_away_threshold = 0.15

        self.distance_history = deque(maxlen=8)
        self.range_rate = 0.0

        # ==================================================
        # STEERING
        # ==================================================

        self.max_steering_angle = 0.35

        # Normal tracking.
        self.steering_gain = 1.35

        # Avoidance.
        self.avoid_steering = 0.30

        # Recovery follows the live detector target more
        # strongly to bring the car back toward track centre.
        self.recover_gain = 1.60

        # Target must be approximately centred before
        # recovery is considered complete.
        self.recover_center_angle = math.radians(5.0)

        # Don't stay in RECOVER forever.
        self.recover_timeout = 2.0

        # Maximum steering change per 20 Hz control cycle.
        # Prevents snapping from avoidance steering to
        # recovery steering.
        self.max_steering_step = 0.045

        self.previous_steering = 0.0

        # ==================================================
        # STATE / TIMING
        # ==================================================

        self.target_timeout = 0.5

        # Commit to an avoidance side for at least this long.
        self.min_avoid_time = 0.70

        self.state = self.NORMAL

        self.target_angle = 0.0
        self.last_target_time = None

        self.front_distance = float('inf')
        self.left_clearance = float('inf')
        self.right_clearance = float('inf')

        self.avoid_start_time = None
        self.recover_start_time = None

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
            'Dynamic Obstacle Avoidance V2.1 + RECOVER started'
        )

    # ==================================================
    # Track detector target
    # ==================================================

    def target_callback(self, msg):

        x = float(msg.point.x)
        y = float(msg.point.y)

        if x <= 0.05:
            return

        self.target_angle = math.atan2(y, x)
        self.last_target_time = self.get_clock().now()

    # ==================================================
    # Robust LiDAR sector clearance
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

        # 10th percentile is more robust than a single
        # minimum LiDAR return.
        index = int(0.10 * (len(values) - 1))

        return values[index]

    # ==================================================
    # LiDAR + temporal range-rate estimation
    # ==================================================

    def scan_callback(self, scan):

        now = self.get_clock().now()

        self.front_distance = self.sector_clearance(
            scan,
            -10.0,
            10.0
        )

        self.left_clearance = self.sector_clearance(
            scan,
            15.0,
            70.0
        )

        self.right_clearance = self.sector_clearance(
            scan,
            -70.0,
            -15.0
        )

        if math.isfinite(self.front_distance):

            t = now.nanoseconds / 1e9

            self.distance_history.append(
                (t, self.front_distance)
            )

            if len(self.distance_history) >= 4:

                old_t, old_d = self.distance_history[0]
                new_t, new_d = self.distance_history[-1]

                dt = new_t - old_t

                if dt > 0.05:

                    raw_rate = (
                        new_d - old_d
                    ) / dt

                    raw_rate = max(
                        -3.0,
                        min(3.0, raw_rate)
                    )

                    self.range_rate = (
                        0.70 * self.range_rate +
                        0.30 * raw_rate
                    )

        else:

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
            self.EMERGENCY_STOP: 'STOP',
            self.RECOVER: 'RECOVER'
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

        elif new_state == self.RECOVER:

            self.recover_start_time = (
                self.get_clock().now()
            )

    def avoidance_time(self):

        if self.avoid_start_time is None:
            return 0.0

        return (
            self.get_clock().now() -
            self.avoid_start_time
        ).nanoseconds / 1e9

    def recovery_time(self):

        if self.recover_start_time is None:
            return 0.0

        return (
            self.get_clock().now() -
            self.recover_start_time
        ).nanoseconds / 1e9

    # ==================================================
    # Choose safer escape side
    # ==================================================

    def choose_escape_side(self):

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

        # If both sides are approximately equal,
        # prefer the current track-detector target.
        if self.target_angle >= 0.0:
            return self.AVOID_LEFT

        return self.AVOID_RIGHT

    # ==================================================
    # Steering rate limiter
    # ==================================================

    def smooth_steering(self, desired):

        difference = (
            desired -
            self.previous_steering
        )

        difference = max(
            -self.max_steering_step,
            min(
                self.max_steering_step,
                difference
            )
        )

        output = (
            self.previous_steering +
            difference
        )

        output = max(
            -self.max_steering_angle,
            min(
                self.max_steering_angle,
                output
            )
        )

        self.previous_steering = output

        return output

    # ==================================================
    # Main controller
    # ==================================================

    def control_loop(self):

        msg = Control()

        # --------------------------------------------------
        # Target freshness
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

        # ==================================================
        # STATE MACHINE
        # ==================================================

        # --------------------------------------------------
        # Absolute emergency stop always has priority
        # --------------------------------------------------

        if (
            self.front_distance <
            self.stop_distance
        ):

            self.set_state(
                self.EMERGENCY_STOP
            )

        # --------------------------------------------------
        # Emergency recovery
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
        # Normal driving
        # --------------------------------------------------

        elif self.state == self.NORMAL:

            # React earlier to an approaching obstacle.
            if (
                approaching and
                self.front_distance <
                self.dynamic_avoid_distance
            ):

                self.set_state(
                    self.choose_escape_side()
                )

            elif (
                self.front_distance <
                self.avoid_distance
            ):

                self.set_state(
                    self.choose_escape_side()
                )

        # --------------------------------------------------
        # Committed obstacle avoidance
        # --------------------------------------------------

        elif self.state in (
            self.AVOID_LEFT,
            self.AVOID_RIGHT
        ):

            if (
                self.avoidance_time() >=
                self.min_avoid_time
            ):

                # Obstacle passed -> DON'T jump directly
                # back to NORMAL. Enter RECOVER first.
                if (
                    self.front_distance >
                    self.clear_distance
                ):

                    self.set_state(
                        self.RECOVER
                    )

                elif (
                    moving_away and
                    self.front_distance >
                    self.avoid_distance
                ):

                    self.set_state(
                        self.RECOVER
                    )

        # --------------------------------------------------
        # Recovery toward track target
        # --------------------------------------------------

        elif self.state == self.RECOVER:

            # A new obstacle appeared while recovering.
            if (
                self.front_distance <
                self.avoid_distance
            ):

                self.set_state(
                    self.choose_escape_side()
                )

            # Target is back close to the centre.
            elif (
                target_valid and
                abs(self.target_angle) <=
                self.recover_center_angle
            ):

                self.set_state(
                    self.NORMAL
                )

            # Safety timeout. Do not get stuck in RECOVER.
            elif (
                self.recovery_time() >=
                self.recover_timeout
            ):

                self.set_state(
                    self.NORMAL
                )

        # ==================================================
        # STEERING
        # ==================================================

        if self.state == self.AVOID_LEFT:

            desired_steering = (
                self.avoid_steering
            )

        elif self.state == self.AVOID_RIGHT:

            desired_steering = (
                -self.avoid_steering
            )

        elif (
            self.state == self.RECOVER and
            target_valid
        ):

            # Follow live detector target strongly.
            # This is what actually brings the vehicle
            # back toward the detected track centre.
            desired_steering = (
                self.recover_gain *
                self.target_angle
            )

        elif target_valid:

            desired_steering = (
                self.steering_gain *
                self.target_angle
            )

        else:

            desired_steering = 0.0

        desired_steering = max(
            -self.max_steering_angle,
            min(
                self.max_steering_angle,
                desired_steering
            )
        )

        steering = self.smooth_steering(
            desired_steering
        )

        # ==================================================
        # VELOCITY
        # ==================================================

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

        elif self.state == self.RECOVER:

            velocity = self.recover_velocity

        elif approaching:

            velocity = self.approach_velocity

        else:

            velocity = self.drive_velocity

        # ==================================================
        # Publish Autoware Control
        # ==================================================

        msg.lateral.steering_tire_angle = steering
        msg.longitudinal.velocity = velocity

        self.control_pub.publish(msg)

        # ==================================================
        # Diagnostics
        # ==================================================

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
            f'target={math.degrees(self.target_angle):+.1f}deg | '
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

    # Explicit stop.
    stop = Control()

    stop.longitudinal.velocity = 0.0
    stop.lateral.steering_tire_angle = 0.0

    node.control_pub.publish(stop)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
