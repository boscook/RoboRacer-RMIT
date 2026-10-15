#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64, String


class ControllerSelector(Node):

    def __init__(self):
        super().__init__('roboracer_controller_selector')

        self.mode = 'STOP'

        self.exploration_speed = 0.0
        self.exploration_steering = 0.0

        self.learned_speed = 0.0
        self.learned_steering = 0.0

        # --------------------------------------------------
        # Mode
        # --------------------------------------------------

        self.create_subscription(
            String,
            '/roboracer/drive_mode',
            self.mode_callback,
            10
        )

        # --------------------------------------------------
        # Exploration controller inputs
        # --------------------------------------------------

        self.create_subscription(
            Float64,
            '/roboracer/exploration/speed',
            self.exploration_speed_callback,
            10
        )

        self.create_subscription(
            Float64,
            '/roboracer/exploration/steering',
            self.exploration_steering_callback,
            10
        )

        # --------------------------------------------------
        # Learned trajectory inputs
        # --------------------------------------------------

        self.create_subscription(
            Float64,
            '/roboracer/follower/speed',
            self.learned_speed_callback,
            10
        )

        self.create_subscription(
            Float64,
            '/roboracer/follower/steering',
            self.learned_steering_callback,
            10
        )

        # --------------------------------------------------
        # Selected output
        # --------------------------------------------------

        self.speed_pub = self.create_publisher(
            Float64,
            '/roboracer/selected/speed',
            10
        )

        self.steering_pub = self.create_publisher(
            Float64,
            '/roboracer/selected/steering',
            10
        )

        self.active_pub = self.create_publisher(
            String,
            '/roboracer/selected/controller',
            10
        )

        self.timer = self.create_timer(
            0.05,
            self.publish_selected
        )

        self.get_logger().warn(
            'Controller Selector started'
        )

        self.get_logger().warn(
            'Initial controller: STOP'
        )

    def mode_callback(self, msg):

        new_mode = msg.data

        if new_mode != self.mode:

            self.get_logger().warn(
                f'MODE: {self.mode} -> {new_mode}'
            )

            self.mode = new_mode

    def exploration_speed_callback(self, msg):
        self.exploration_speed = float(msg.data)

    def exploration_steering_callback(self, msg):
        self.exploration_steering = float(msg.data)

    def learned_speed_callback(self, msg):
        self.learned_speed = float(msg.data)

    def learned_steering_callback(self, msg):
        self.learned_steering = float(msg.data)

    def publish_selected(self):

        speed = 0.0
        steering = 0.0
        controller = 'STOP'

        if self.mode == 'EXPLORATION':

            speed = self.exploration_speed
            steering = self.exploration_steering
            controller = 'EXPLORATION'

        elif self.mode == 'LEARNED':

            speed = self.learned_speed
            steering = self.learned_steering
            controller = 'LEARNED'

        speed_msg = Float64()
        speed_msg.data = float(speed)

        steering_msg = Float64()
        steering_msg.data = float(steering)

        active_msg = String()
        active_msg.data = controller

        self.speed_pub.publish(speed_msg)
        self.steering_pub.publish(steering_msg)
        self.active_pub.publish(active_msg)


def main(args=None):

    rclpy.init(args=args)

    node = ControllerSelector()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
