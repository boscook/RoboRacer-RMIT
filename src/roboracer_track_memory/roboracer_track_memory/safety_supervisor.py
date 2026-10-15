#!/usr/bin/env python3

import math
from collections import deque

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64, String


class SafetySupervisor(Node):

    def __init__(self):

        super().__init__('roboracer_safety_supervisor')

        # ==================================================
        # LIDAR SECTORS
        # ==================================================

        # Critical collision zone.
        self.front_half_angle = math.radians(25.0)

        # Wider awareness sector.
        self.wide_half_angle = math.radians(50.0)

        # ==================================================
        # FAIL-SAFE
        # ==================================================

        # Hokuyo normally runs at ~40 Hz.
        # If scan becomes stale, stop.
        self.scan_timeout = 0.25

        # ==================================================
        # DYNAMIC RACE SAFETY PARAMETERS
        # ==================================================

        # Absolute minimum emergency distance.
        self.minimum_emergency_distance = 0.40

        # Approximate usable braking deceleration.
        #
        # Conservative starting estimate.
        # Tune after real braking-distance tests.
        self.braking_deceleration = 4.0  # m/s^2

        # Controller / actuator / processing reaction allowance.
        self.reaction_time = 0.18  # seconds

        # Physical margin after theoretical stopping point.
        self.stop_margin = 0.30  # metres

        # Extra zone before emergency braking begins.
        self.minimum_slow_margin = 0.60

        # Maximum requested racing speed expected.
        self.maximum_expected_speed = 4.0

        # ==================================================
        # INPUT STATE
        # ==================================================

        self.requested_speed = 0.0
        self.requested_steering = 0.0

        self.last_scan_time = None

        self.front_distance = float('inf')
        self.wide_distance = float('inf')

        # ==================================================
        # CLOSING RATE
        # ==================================================

        self.front_history = deque(maxlen=8)
        self.closing_rate = 0.0

        # ==================================================
        # OUTPUT STATE
        # ==================================================

        self.state = 'NO_SCAN'

        self.dynamic_emergency_distance = (
            self.minimum_emergency_distance
        )

        self.dynamic_slow_distance = (
            self.minimum_emergency_distance +
            self.minimum_slow_margin
        )

        # ==================================================
        # SUBSCRIBERS
        # ==================================================

        self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.create_subscription(
            Float64,
            '/roboracer/selected/speed',
            self.speed_callback,
            10
        )

        self.create_subscription(
            Float64,
            '/roboracer/selected/steering',
            self.steering_callback,
            10
        )

        # ==================================================
        # PUBLISHERS
        # ==================================================

        self.safe_speed_pub = self.create_publisher(
            Float64,
            '/roboracer/safety/speed',
            10
        )

        self.safe_steering_pub = self.create_publisher(
            Float64,
            '/roboracer/safety/steering',
            10
        )

        self.state_pub = self.create_publisher(
            String,
            '/roboracer/safety/state',
            10
        )

        self.front_pub = self.create_publisher(
            Float64,
            '/roboracer/safety/front_distance',
            10
        )

        # Useful new diagnostics.
        self.emergency_distance_pub = self.create_publisher(
            Float64,
            '/roboracer/safety/emergency_distance',
            10
        )

        self.slow_distance_pub = self.create_publisher(
            Float64,
            '/roboracer/safety/slow_distance',
            10
        )

        self.closing_rate_pub = self.create_publisher(
            Float64,
            '/roboracer/safety/closing_rate',
            10
        )

        # Independent 20 Hz watchdog.
        self.timer = self.create_timer(
            0.05,
            self.control
        )

        self.get_logger().warn(
            '============================================'
        )

        self.get_logger().warn(
            'RoboRacer DYNAMIC RACE SAFETY started'
        )

        self.get_logger().warn(
            'Speed-dependent braking distance ENABLED'
        )

        self.get_logger().warn(
            'NO_SCAN / stale scan => MOTOR STOP'
        )

        self.get_logger().warn(
            'Automatic reverse remains BLOCKED'
        )

        self.get_logger().warn(
            '============================================'
        )

    # ==================================================
    # COMMAND INPUTS
    # ==================================================

    def speed_callback(self, msg):

        self.requested_speed = float(
            msg.data
        )

    def steering_callback(self, msg):

        self.requested_steering = float(
            msg.data
        )

    # ==================================================
    # ROBUST LIDAR DISTANCE
    # ==================================================

    @staticmethod
    def robust_distance(values):

        if not values:
            return float('inf')

        values = sorted(values)

        # Roughly 5th percentile.
        #
        # Safer than mean, less noise-sensitive
        # than absolute minimum.
        index = int(
            0.05 *
            (len(values) - 1)
        )

        return float(
            values[index]
        )

    # ==================================================
    # LIDAR
    # ==================================================

    def scan_callback(self, msg):

        front_values = []
        wide_values = []

        angle = msg.angle_min

        for distance in msg.ranges:

            if (
                math.isfinite(distance) and
                distance >= msg.range_min and
                distance <= msg.range_max
            ):

                if (
                    abs(angle) <=
                    self.wide_half_angle
                ):

                    wide_values.append(
                        float(distance)
                    )

                if (
                    abs(angle) <=
                    self.front_half_angle
                ):

                    front_values.append(
                        float(distance)
                    )

            angle += msg.angle_increment

        self.front_distance = (
            self.robust_distance(
                front_values
            )
        )

        self.wide_distance = (
            self.robust_distance(
                wide_values
            )
        )

        now = self.get_clock().now()

        self.last_scan_time = now

        # ==================================================
        # INDEPENDENT CLOSING-RATE ESTIMATE
        # ==================================================

        if math.isfinite(
            self.front_distance
        ):

            timestamp = (
                now.nanoseconds /
                1e9
            )

            self.front_history.append(
                (
                    timestamp,
                    self.front_distance
                )
            )

            if len(
                self.front_history
            ) >= 4:

                old_t, old_d = (
                    self.front_history[0]
                )

                new_t, new_d = (
                    self.front_history[-1]
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
                        -8.0,
                        min(
                            8.0,
                            raw_rate
                        )
                    )

                    # Low-pass filter.
                    self.closing_rate = (
                        0.70 *
                        self.closing_rate +
                        0.30 *
                        raw_rate
                    )

        else:

            self.front_history.clear()
            self.closing_rate = 0.0

        # React immediately to a fresh scan.
        self.control()

    # ==================================================
    # DYNAMIC STOPPING DISTANCE
    # ==================================================

    def calculate_dynamic_distances(self):

        # Only forward speed matters.
        velocity = max(
            0.0,
            min(
                self.maximum_expected_speed,
                self.requested_speed
            )
        )

        # ----------------------------------------------
        # Reaction distance
        # ----------------------------------------------

        reaction_distance = (
            velocity *
            self.reaction_time
        )

        # ----------------------------------------------
        # Theoretical braking distance
        #
        #               v^2
        # d = ---------------------
        #       2 * deceleration
        # ----------------------------------------------

        braking_distance = (
            velocity *
            velocity
        ) / (
            2.0 *
            self.braking_deceleration
        )

        emergency_distance = (
            reaction_distance +
            braking_distance +
            self.stop_margin
        )

        emergency_distance = max(
            self.minimum_emergency_distance,
            emergency_distance
        )

        # ==================================================
        # DYNAMIC OBJECT COMPENSATION
        #
        # Negative rate means obstacle is getting closer.
        # ==================================================

        if self.closing_rate < 0.0:

            approaching_speed = abs(
                self.closing_rate
            )

            # Add another reaction-time term.
            emergency_distance += (
                approaching_speed *
                0.20
            )

        # ==================================================
        # SLOWING ZONE
        # ==================================================

        slow_margin = max(
            self.minimum_slow_margin,
            0.40 *
            velocity
        )

        slow_distance = (
            emergency_distance +
            slow_margin
        )

        self.dynamic_emergency_distance = (
            emergency_distance
        )

        self.dynamic_slow_distance = (
            slow_distance
        )

    # ==================================================
    # CONTROL
    # ==================================================

    def control(self):

        now = self.get_clock().now()

        # ==================================================
        # NO SCAN
        # ==================================================

        if self.last_scan_time is None:

            self.publish_command(
                0.0,
                0.0,
                'NO_SCAN'
            )

            return

        age = (
            now -
            self.last_scan_time
        ).nanoseconds / 1e9

        # ==================================================
        # STALE SCAN
        # ==================================================

        if (
            age >
            self.scan_timeout
        ):

            self.publish_command(
                0.0,
                0.0,
                'SCAN_TIMEOUT'
            )

            return

        requested_speed = float(
            self.requested_speed
        )

        requested_steering = float(
            self.requested_steering
        )

        # ==================================================
        # REVERSE
        #
        # Still intentionally blocked because current
        # safety system only protects forward motion.
        # ==================================================

        if requested_speed < 0.0:

            self.publish_command(
                0.0,
                requested_steering,
                'REVERSE_BLOCKED'
            )

            return

        # ==================================================
        # UPDATE DYNAMIC DISTANCES
        # ==================================================

        self.calculate_dynamic_distances()

        # ==================================================
        # EMERGENCY STOP
        # ==================================================

        if (
            self.front_distance <=
            self.dynamic_emergency_distance
        ):

            self.publish_command(
                0.0,
                requested_steering,
                'EMERGENCY_STOP'
            )

            return

        # ==================================================
        # SLOW ZONE
        # ==================================================

        if (
            self.front_distance <
            self.dynamic_slow_distance
        ):

            span = (
                self.dynamic_slow_distance -
                self.dynamic_emergency_distance
            )

            if span <= 0.001:

                factor = 0.0

            else:

                factor = (
                    self.front_distance -
                    self.dynamic_emergency_distance
                ) / span

            factor = max(
                0.0,
                min(
                    1.0,
                    factor
                )
            )

            # Smoothly reduce speed toward zero
            # as braking boundary approaches.
            safe_speed = (
                requested_speed *
                factor
            )

            self.publish_command(
                safe_speed,
                requested_steering,
                'SLOW'
            )

            return

        # ==================================================
        # CLEAR
        # ==================================================

        self.publish_command(
            requested_speed,
            requested_steering,
            'CLEAR'
        )

    # ==================================================
    # OUTPUT
    # ==================================================

    def publish_command(
        self,
        speed,
        steering,
        state
    ):

        speed_msg = Float64()
        speed_msg.data = float(
            speed
        )

        steering_msg = Float64()
        steering_msg.data = float(
            steering
        )

        state_msg = String()
        state_msg.data = state

        front_msg = Float64()
        front_msg.data = float(
            self.front_distance
        )

        emergency_msg = Float64()
        emergency_msg.data = float(
            self.dynamic_emergency_distance
        )

        slow_msg = Float64()
        slow_msg.data = float(
            self.dynamic_slow_distance
        )

        rate_msg = Float64()
        rate_msg.data = float(
            self.closing_rate
        )

        self.safe_speed_pub.publish(
            speed_msg
        )

        self.safe_steering_pub.publish(
            steering_msg
        )

        self.state_pub.publish(
            state_msg
        )

        self.front_pub.publish(
            front_msg
        )

        self.emergency_distance_pub.publish(
            emergency_msg
        )

        self.slow_distance_pub.publish(
            slow_msg
        )

        self.closing_rate_pub.publish(
            rate_msg
        )

        if state != self.state:

            self.get_logger().warn(
                f'SAFETY: {self.state} -> {state} | '
                f'front={self.front_distance:.2f}m | '
                f'emergency={self.dynamic_emergency_distance:.2f}m | '
                f'slow={self.dynamic_slow_distance:.2f}m | '
                f'rate={self.closing_rate:+.2f}m/s | '
                f'request={self.requested_speed:.2f}m/s | '
                f'output={speed:.2f}m/s'
            )

            self.state = state


def main(args=None):

    rclpy.init(
        args=args
    )

    node = SafetySupervisor()

    try:

        rclpy.spin(
            node
        )

    except KeyboardInterrupt:

        pass

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
