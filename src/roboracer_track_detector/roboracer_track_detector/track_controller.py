#!/usr/bin/env python3

import math
from collections import deque

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64


class TrackController(Node):

    NORMAL = 0
    AVOID_LEFT = 1
    AVOID_RIGHT = 2
    EMERGENCY_STOP = 3
    RECOVER = 4

    def __init__(self):
        super().__init__('roboracer_track_controller')

        # ==================================================
        # RACE SPEED SETTINGS
        # Adapted from RoboRacer Milestone 5
        # ==================================================

        self.max_speed = 1.0

        self.straight_speed = 3.2
        self.fast_corner_speed = 2.5
        self.medium_corner_speed = 2.0
        self.sharp_corner_speed = 1.5

        self.avoid_speed = 0.5
        self.recover_speed = 0.4

        # ==================================================
        # DYNAMIC SPEED / CLEARANCE
        # ==================================================

        self.front_clear_for_full_speed = 4.0
        self.front_medium_distance = 2.5
        self.front_close_distance = 1.4

        # ==================================================
        # OBSTACLE DISTANCES
        # ==================================================

        self.avoid_distance = 0.60
        self.dynamic_avoid_distance = 0.90

        self.stop_distance = 0.28
        self.clear_distance = 0.80

        # ==================================================
        # RANGE RATE
        # ==================================================

        self.approaching_threshold = -0.20
        self.moving_away_threshold = 0.20

        self.distance_history = deque(maxlen=8)
        self.range_rate = 0.0

        # ==================================================
        # STEERING
        # ==================================================

        self.max_steering_angle = 0.35

        self.steering_gain = 0.80
        self.avoid_steering = 0.30
        self.recover_gain = 1.30

        # Slightly faster steering response than old version
        self.max_steering_step = 0.060
        self.previous_steering = 0.0

        self.recover_center_angle = math.radians(5.0)
        self.recover_timeout = 1.5

        # ==================================================
        # STATE
        # ==================================================

        self.target_timeout = 0.30
        self.min_avoid_time = 0.35

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

        self.speed_pub = self.create_publisher(
            Float64,
            '/roboracer/exploration/speed',
            10
        )

        self.steering_pub = self.create_publisher(
            Float64,
            '/roboracer/exploration/steering',
            10
        )

        self.timer = self.create_timer(
            0.05,
            self.control_loop
        )

        self.get_logger().warn(
            'RoboRacer race-speed exploration controller started'
        )

    # ==================================================
    # TARGET
    # ==================================================

    def target_callback(self, msg):

        x = float(msg.point.x)
        y = float(msg.point.y)

        if x <= 0.05:
            return

        self.target_angle = math.atan2(
            y,
            x
        )

        self.last_target_time = (
            self.get_clock().now()
        )

    # ==================================================
    # LIDAR SECTOR
    # ==================================================

    def sector_clearance(
        self,
        scan,
        angle_min_deg,
        angle_max_deg
    ):

        values = []

        amin = math.radians(
            angle_min_deg
        )

        amax = math.radians(
            angle_max_deg
        )

        for i, distance in enumerate(
            scan.ranges
        ):

            angle = (
                scan.angle_min +
                i * scan.angle_increment
            )

            if (
                angle < amin or
                angle > amax
            ):
                continue

            if not math.isfinite(distance):
                continue

            if (
                distance < scan.range_min or
                distance > scan.range_max
            ):
                continue

            values.append(
                float(distance)
            )

        if not values:
            return float('inf')

        values.sort()

        # 10th percentile rather than one noisy minimum
        index = int(
            0.10 *
            (len(values) - 1)
        )

        return values[index]

    # ==================================================
    # LIDAR
    # ==================================================

    def scan_callback(self, scan):

        now = (
            self.get_clock().now()
        )

        self.front_distance = (
            self.sector_clearance(
                scan,
                -10.0,
                10.0
            )
        )

        self.left_clearance = (
            self.sector_clearance(
                scan,
                15.0,
                70.0
            )
        )

        self.right_clearance = (
            self.sector_clearance(
                scan,
                -70.0,
                -15.0
            )
        )

        if math.isfinite(
            self.front_distance
        ):

            t = (
                now.nanoseconds /
                1e9
            )

            self.distance_history.append(
                (
                    t,
                    self.front_distance
                )
            )

            if len(
                self.distance_history
            ) >= 4:

                old_t, old_d = (
                    self.distance_history[0]
                )

                new_t, new_d = (
                    self.distance_history[-1]
                )

                dt = (
                    new_t -
                    old_t
                )

                if dt > 0.05:

                    raw_rate = (
                        new_d -
                        old_d
                    ) / dt

                    raw_rate = max(
                        -5.0,
                        min(
                            5.0,
                            raw_rate
                        )
                    )

                    self.range_rate = (
                        0.70 *
                        self.range_rate +
                        0.30 *
                        raw_rate
                    )

        else:

            self.distance_history.clear()
            self.range_rate = 0.0

    # ==================================================
    # STATE
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

    def set_state(
        self,
        new_state
    ):

        if new_state == self.state:
            return

        old = self.state_name()

        self.state = new_state

        self.get_logger().warn(
            f'STATE: {old} -> {self.state_name()}'
        )

        now = (
            self.get_clock().now()
        )

        if new_state in (
            self.AVOID_LEFT,
            self.AVOID_RIGHT
        ):
            self.avoid_start_time = now

        elif new_state == self.RECOVER:
            self.recover_start_time = now

    def elapsed(
        self,
        start_time
    ):

        if start_time is None:
            return 0.0

        return (
            self.get_clock().now() -
            start_time
        ).nanoseconds / 1e9

    # ==================================================
    # AVOIDANCE SIDE
    # ==================================================

    def choose_escape_side(self):

        margin = 0.08

        if (
            self.left_clearance >
            self.right_clearance +
            margin
        ):
            return self.AVOID_LEFT

        if (
            self.right_clearance >
            self.left_clearance +
            margin
        ):
            return self.AVOID_RIGHT

        if self.target_angle >= 0.0:
            return self.AVOID_LEFT

        return self.AVOID_RIGHT

    # ==================================================
    # STEERING SMOOTHING
    # ==================================================

    def smooth_steering(
        self,
        desired
    ):

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

        self.previous_steering = (
            output
        )

        return output

    # ==================================================
    # RACE SPEED
    # ==================================================

    def race_speed(
        self,
        steering
    ):

        angle_deg = abs(
            math.degrees(
                steering
            )
        )

        # Milestone-5-inspired speed bands.
        if angle_deg <= 1.5:

            speed = (
                self.straight_speed
            )

            # Extra acceleration on a very clear straight.
            if math.isfinite(
                self.front_distance
            ):

                bonus = max(
                    0.0,
                    min(
                        0.8,
                        0.20 *
                        (
                            self.front_distance -
                            2.0
                        )
                    )
                )

                speed += bonus

        elif angle_deg <= 7.0:

            speed = (
                self.fast_corner_speed
            )

        elif angle_deg <= 15.0:

            speed = (
                self.medium_corner_speed
            )

        else:

            speed = (
                self.sharp_corner_speed
            )

        # ----------------------------------------------
        # Clearance based speed limiting
        # ----------------------------------------------

        if math.isfinite(
            self.front_distance
        ):

            if (
                self.front_distance <
                self.front_close_distance
            ):

                speed = min(
                    speed,
                    1.0
                )

            elif (
                self.front_distance <
                self.front_medium_distance
            ):

                speed = min(
                    speed,
                    1.8
                )

            elif (
                self.front_distance <
                self.front_clear_for_full_speed
            ):

                speed = min(
                    speed,
                    2.8
                )

        # ----------------------------------------------
        # Closing object
        # ----------------------------------------------

        if (
            self.range_rate <
            -0.50
        ):

            speed = min(
                speed,
                1.5
            )

        elif (
            self.range_rate <
            -0.20
        ):

            speed = min(
                speed,
                2.0
            )

        return max(
            0.0,
            min(
                self.max_speed,
                speed
            )
        )

    # ==================================================
    # CONTROL
    # ==================================================

    def control_loop(self):

        target_valid = False

        if (
            self.last_target_time
            is not None
        ):

            age = (
                self.get_clock().now() -
                self.last_target_time
            ).nanoseconds / 1e9

            target_valid = (
                age <=
                self.target_timeout
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

        if (
            self.state !=
            self.EMERGENCY_STOP and
            self.front_distance <
            self.stop_distance
        ):

            self.set_state(
                self.EMERGENCY_STOP
            )

        elif (
            self.state ==
            self.EMERGENCY_STOP
        ):

            if (
                self.front_distance >
                self.stop_distance +
                0.30
            ):

                self.set_state(
                    self.choose_escape_side()
                )

        elif (
            self.state ==
            self.NORMAL
        ):

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

        elif self.state in (
            self.AVOID_LEFT,
            self.AVOID_RIGHT
        ):

            if (
                self.elapsed(
                    self.avoid_start_time
                ) >=
                self.min_avoid_time
            ):

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

        elif (
            self.state ==
            self.RECOVER
        ):

            if (
                self.front_distance <
                self.avoid_distance
            ):

                self.set_state(
                    self.choose_escape_side()
                )

            elif (
                target_valid and
                abs(
                    self.target_angle
                ) <=
                self.recover_center_angle
            ):

                self.set_state(
                    self.NORMAL
                )

            elif (
                self.elapsed(
                    self.recover_start_time
                ) >=
                self.recover_timeout
            ):

                self.set_state(
                    self.NORMAL
                )

        # ==================================================
        # STEERING
        # ==================================================

        if (
            self.state ==
            self.AVOID_LEFT
        ):

            desired_steering = (
                self.avoid_steering
            )

        elif (
            self.state ==
            self.AVOID_RIGHT
        ):

            desired_steering = (
                -self.avoid_steering
            )

        elif (
            self.state ==
            self.RECOVER and
            target_valid
        ):

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

        steering = (
            self.smooth_steering(
                desired_steering
            )
        )

        # ==================================================
        # SPEED
        # ==================================================

        if (
            self.state ==
            self.EMERGENCY_STOP
        ):

            velocity = 0.0

        elif not target_valid:

            velocity = 0.0

        elif self.state in (
            self.AVOID_LEFT,
            self.AVOID_RIGHT
        ):

            velocity = (
                self.avoid_speed
            )

        elif (
            self.state ==
            self.RECOVER
        ):

            velocity = (
                self.recover_speed
            )

        else:

            velocity = (
                self.race_speed(
                    steering
                )
            )

        # ==================================================
        # OUTPUT
        # ==================================================

        speed_msg = Float64()
        speed_msg.data = float(
            velocity
        )

        steering_msg = Float64()
        steering_msg.data = float(
            steering
        )

        self.speed_pub.publish(
            speed_msg
        )

        self.steering_pub.publish(
            steering_msg
        )

        self.get_logger().info(
            f'{self.state_name()} | '
            f'front={self.front_distance:.2f}m | '
            f'rate={self.range_rate:+.2f}m/s | '
            f'target={math.degrees(self.target_angle):+.1f}deg | '
            f'v={velocity:.2f}m/s | '
            f'steer={math.degrees(steering):+.1f}deg',
            throttle_duration_sec=0.25
        )


def main(args=None):

    rclpy.init(
        args=args
    )

    node = TrackController()

    try:

        rclpy.spin(
            node
        )

    except KeyboardInterrupt:

        pass

    stop_speed = Float64()
    stop_speed.data = 0.0

    stop_steering = Float64()
    stop_steering.data = 0.0

    node.speed_pub.publish(
        stop_speed
    )

    node.steering_pub.publish(
        stop_steering
    )

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
