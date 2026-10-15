#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from std_msgs.msg import UInt32, Bool


class LapDetector(Node):

    def __init__(self):
        super().__init__('roboracer_lap_detector')

        # ==================================================
        # CONFIGURATION
        # ==================================================

        # Must leave the start area before a lap can finish.
        self.leave_start_distance = 1.50

        # Return within this distance of starting position.
        self.finish_distance = 0.60

        # Heading must approximately match starting heading.
        self.finish_heading_tolerance = math.radians(35.0)

        # Prevent tiny loops / accidental lap detection.
        self.minimum_lap_distance = 5.0

        # ==================================================
        # STATE
        # ==================================================

        self.start_x = None
        self.start_y = None
        self.start_yaw = None

        self.previous_x = None
        self.previous_y = None

        self.total_distance = 0.0
        self.lap_distance = 0.0

        self.left_start_area = False
        self.lap_count = 0

        # ==================================================
        # ROS
        # ==================================================

        self.pose_sub = self.create_subscription(
            Odometry,
            '/localization/kinematic_state',
            self.pose_callback,
            20
        )

        self.lap_pub = self.create_publisher(
            UInt32,
            '/roboracer/lap_count',
            10
        )

        self.complete_pub = self.create_publisher(
            Bool,
            '/roboracer/lap_complete',
            10
        )

        self.get_logger().info(
            'RoboRacer automatic lap detector started'
        )

    # ==================================================
    # Helpers
    # ==================================================

    @staticmethod
    def quaternion_to_yaw(q):

        siny_cosp = 2.0 * (
            q.w * q.z +
            q.x * q.y
        )

        cosy_cosp = 1.0 - 2.0 * (
            q.y * q.y +
            q.z * q.z
        )

        return math.atan2(
            siny_cosp,
            cosy_cosp
        )

    @staticmethod
    def normalize_angle(angle):

        while angle > math.pi:
            angle -= 2.0 * math.pi

        while angle < -math.pi:
            angle += 2.0 * math.pi

        return angle

    # ==================================================
    # Pose
    # ==================================================

    def pose_callback(self, msg):

        x = float(msg.pose.pose.position.x)
        y = float(msg.pose.pose.position.y)

        yaw = self.quaternion_to_yaw(
            msg.pose.pose.orientation
        )

        # ----------------------------------------------
        # First valid pose becomes starting position
        # ----------------------------------------------

        if self.start_x is None:

            self.start_x = x
            self.start_y = y
            self.start_yaw = yaw

            self.previous_x = x
            self.previous_y = y

            self.get_logger().info(
                f'START SAVED: '
                f'x={x:.2f} '
                f'y={y:.2f} '
                f'yaw={math.degrees(yaw):.1f} deg'
            )

            return

        # ----------------------------------------------
        # Distance travelled
        # ----------------------------------------------

        step = math.hypot(
            x - self.previous_x,
            y - self.previous_y
        )

        # Reject impossible localization jumps.
        if step < 1.0:

            self.total_distance += step
            self.lap_distance += step

        self.previous_x = x
        self.previous_y = y

        # ----------------------------------------------
        # Distance from start
        # ----------------------------------------------

        start_distance = math.hypot(
            x - self.start_x,
            y - self.start_y
        )

        # Must genuinely leave start first.
        if (
            not self.left_start_area and
            start_distance >= self.leave_start_distance
        ):

            self.left_start_area = True

            self.get_logger().info(
                'LEFT START AREA - lap detection armed'
            )

        # ----------------------------------------------
        # Lap completion
        # ----------------------------------------------

        if not self.left_start_area:
            return

        if self.lap_distance < self.minimum_lap_distance:
            return

        heading_error = abs(
            self.normalize_angle(
                yaw - self.start_yaw
            )
        )

        if (
            start_distance <= self.finish_distance and
            heading_error <= self.finish_heading_tolerance
        ):

            self.lap_count += 1

            self.get_logger().warn(
                '===================================='
            )

            self.get_logger().warn(
                f'LAP {self.lap_count} COMPLETE | '
                f'distance={self.lap_distance:.2f} m | '
                f'finish_error={start_distance:.2f} m | '
                f'heading_error='
                f'{math.degrees(heading_error):.1f} deg'
            )

            self.get_logger().warn(
                '===================================='
            )

            count = UInt32()
            count.data = self.lap_count
            self.lap_pub.publish(count)

            complete = Bool()
            complete.data = True
            self.complete_pub.publish(complete)

            # Start measuring next lap.
            self.lap_distance = 0.0

            # Require leaving the start area again.
            self.left_start_area = False


def main(args=None):

    rclpy.init(args=args)

    node = LapDetector()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
