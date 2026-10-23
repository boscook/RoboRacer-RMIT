#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64, Bool, String
from autoware_control_msgs.msg import Control


class SafeControlAdapter(Node):

    def __init__(self):

        super().__init__('roboracer_safe_control_adapter')

        # ==================================================
        # INPUT STATE
        # ==================================================

        self.safe_speed = 0.0
        self.safe_steering = 0.0
        self.safety_state = 'UNKNOWN'

        # Manual master arm.
        self.manual_enable = False

        # Permission from mode/state manager.
        self.automatic_enable = False

        # ==================================================
        # INPUTS
        # ==================================================

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

        # Physical/master ARM.
        self.create_subscription(
            Bool,
            '/roboracer/drive_enable',
            self.manual_enable_callback,
            10
        )

        # State-machine permission.
        self.create_subscription(
            Bool,
            '/roboracer/automatic_drive_enable',
            self.automatic_enable_callback,
            10
        )

        # ==================================================
        # OUTPUT
        # ==================================================

        self.control_pub = self.create_publisher(
            Control,
            '/control/command/control_cmd',
            10
        )

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
            'MANUAL ARM = FALSE'
        )

        self.get_logger().warn(
            'AUTOMATIC PERMISSION = FALSE'
        )

        self.get_logger().warn(
            'Vehicle cannot move until BOTH are TRUE'
        )

        self.get_logger().warn(
            '========================================'
        )

    # ==================================================
    # CALLBACKS
    # ==================================================

    def speed_callback(self, msg):

        self.safe_speed = float(
            msg.data
        )

    def steering_callback(self, msg):

        self.safe_steering = float(
            msg.data
        )

    def safety_callback(self, msg):

        self.safety_state = msg.data

    def manual_enable_callback(self, msg):

        new_state = bool(
            msg.data
        )

        if new_state != self.manual_enable:

            self.manual_enable = (
                new_state
            )

            self.get_logger().warn(
                f'MANUAL ARM: '
                f'{self.manual_enable}'
            )

    def automatic_enable_callback(
        self,
        msg
    ):

        new_state = bool(
            msg.data
        )

        if (
            new_state !=
            self.automatic_enable
        ):

            self.automatic_enable = (
                new_state
            )

            self.get_logger().warn(
                f'AUTOMATIC PERMISSION: '
                f'{self.automatic_enable}'
            )

    # ==================================================
    # FINAL CONTROL
    # ==================================================

    def publish_control(self):

        msg = Control()

        now = (
            self.get_clock()
            .now()
            .to_msg()
        )

        msg.stamp = now
        msg.lateral.stamp = now
        msg.longitudinal.stamp = now

        # ==================================================
        # STEERING
        # ==================================================

        msg.lateral.steering_tire_angle = float(
            self.safe_steering
        )

        msg.lateral.steering_tire_rotation_rate = 0.0

        msg.lateral.is_defined_steering_tire_rotation_rate = False

        # ==================================================
        # SPEED AUTHORIZATION
        # ==================================================

        allowed_states = {
            'CLEAR',
            'SLOW',
            'REVERSE'
        }

        drive_authorized = (
            self.manual_enable and
            self.automatic_enable and
            self.safety_state in allowed_states
        )

        if drive_authorized:

            if self.safety_state == 'REVERSE':
                velocity = self.safe_speed
            else:
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

        self.control_pub.publish(
            msg
        )


def main(args=None):

    rclpy.init(
        args=args
    )

    node = SafeControlAdapter()

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
