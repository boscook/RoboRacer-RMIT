#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64, Bool, String
from autoware_control_msgs.msg import Control


class SafeControlAdapter(Node):

    def __init__(self):
        super().__init__('roboracer_safe_control_adapter')

        self.safe_speed = 0.0
        self.safe_steering = 0.0

        self.drive_enable = False
        self.safety_state = 'UNKNOWN'

        # --------------------------------------------------
        # Inputs
        # --------------------------------------------------

        self.create_subscription(
            Float64,
            '/roboracer/safety/speed',
            self.speed_callback,
            10
        )

        self.create_subscription(
            Float64,
            '/roboracer/safety/steering',
            self.steering_callback,
            10
        )

        self.create_subscription(
            String,
            '/roboracer/safety/state',
            self.safety_callback,
            10
        )

        self.create_subscription(
            Bool,
            '/roboracer/drive_enable',
            self.enable_callback,
            10
        )

        # --------------------------------------------------
        # Output
        # --------------------------------------------------

        self.control_pub = self.create_publisher(
            Control,
            '/control/command/control_cmd',
            10
        )

        # 20 Hz final command stream.
        self.timer = self.create_timer(
            0.05,
            self.publish_control
        )

        self.get_logger().warn(
            '========================================'
        )
        self.get_logger().warn(
            'Safe Autoware Control Adapter started'
        )
        self.get_logger().warn(
            'DRIVE DISABLED'
        )
        self.get_logger().warn(
            'Waiting for /roboracer/drive_enable=true'
        )
        self.get_logger().warn(
            '========================================'
        )

    def speed_callback(self, msg):
        self.safe_speed = float(msg.data)

    def steering_callback(self, msg):
        self.safe_steering = float(msg.data)

    def safety_callback(self, msg):
        self.safety_state = msg.data

    def enable_callback(self, msg):

        new_state = bool(msg.data)

        if new_state != self.drive_enable:

            self.drive_enable = new_state

            if self.drive_enable:

                self.get_logger().warn(
                    'DRIVE ENABLED'
                )

            else:

                self.get_logger().warn(
                    'DRIVE DISABLED'
                )

    def publish_control(self):

        msg = Control()

        now = self.get_clock().now().to_msg()

        msg.stamp = now

        msg.lateral.stamp = now
        msg.longitudinal.stamp = now

        # --------------------------------------------------
        # Steering
        # --------------------------------------------------

        msg.lateral.steering_tire_angle = float(
            self.safe_steering
        )

        msg.lateral.steering_tire_rotation_rate = 0.0

        msg.lateral.is_defined_steering_tire_rotation_rate = False

        # --------------------------------------------------
        # Velocity
        #
        # Fail-safe conditions:
        #   drive disabled
        #   no scan
        #   stale scan
        #   emergency stop
        #   reverse blocked
        # --------------------------------------------------

        allowed_states = {
            'CLEAR',
            'SLOW'
        }

        if (
            self.drive_enable and
            self.safety_state in allowed_states
        ):

            velocity = max(
                0.0,
                self.safe_speed
            )

        else:

            velocity = 0.0

        msg.longitudinal.velocity = float(
            velocity
        )

        msg.longitudinal.acceleration = 0.0
        msg.longitudinal.jerk = 0.0

        msg.longitudinal.is_defined_acceleration = False
        msg.longitudinal.is_defined_jerk = False

        self.control_pub.publish(msg)


def main(args=None):

    rclpy.init(args=args)

    node = SafeControlAdapter()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
