#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Bool


class ModeManager(Node):

    def __init__(self):
        super().__init__('roboracer_mode_manager')

        self.memory_state = 'UNKNOWN'
        self.mode = 'STOP'

        self.mode_pub = self.create_publisher(
            String,
            '/roboracer/drive_mode',
            10
        )

        self.enable_pub = self.create_publisher(
            Bool,
            '/roboracer/automatic_drive_enable',
            10
        )

        self.create_subscription(
            String,
            '/roboracer/memory_state',
            self.memory_callback,
            10
        )

        self.timer = self.create_timer(
            0.1,
            self.update_mode
        )

        self.get_logger().warn(
            'RoboRacer Mode Manager started'
        )

        self.get_logger().warn(
            'Initial mode: STOP'
        )

    def memory_callback(self, msg):
        self.memory_state = msg.data

    def update_mode(self):

        if self.memory_state == 'EXPLORING':
            new_mode = 'EXPLORATION'

        elif self.memory_state == 'BUILDING_MEMORY':
            new_mode = 'STOP'

        elif self.memory_state in (
            'MEMORY_READY',
            'LEARNING'
        ):
            new_mode = 'LEARNED'

        else:
            new_mode = 'STOP'

        if new_mode != self.mode:

            self.get_logger().warn(
                f'DRIVE MODE: '
                f'{self.mode} -> {new_mode}'
            )

            self.mode = new_mode

        mode_msg = String()
        mode_msg.data = self.mode

        self.mode_pub.publish(
            mode_msg
        )

        # BUILDING_MEMORY / UNKNOWN / failures
        # must never drive.
        enable_msg = Bool()

        enable_msg.data = (
            self.mode in (
                'EXPLORATION',
                'LEARNED'
            )
        )

        self.enable_pub.publish(
            enable_msg
        )


def main(args=None):

    rclpy.init(args=args)

    node = ModeManager()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
