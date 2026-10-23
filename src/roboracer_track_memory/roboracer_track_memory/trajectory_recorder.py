#!/usr/bin/env python3

import csv
import math
from pathlib import Path

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import UInt32


def sector_min(scan, lo_deg, hi_deg, cap=3.0):
    """Smallest valid range in an angular sector (degrees, 0 = straight ahead)."""
    best = cap
    lo, hi = math.radians(lo_deg), math.radians(hi_deg)
    for i, r in enumerate(scan.ranges):
        a = scan.angle_min + i * scan.angle_increment
        if lo <= a <= hi and math.isfinite(r) and scan.range_min < r < best:
            best = r
    return best


class TrajectoryRecorder(Node):

    def __init__(self):
        super().__init__('roboracer_trajectory_recorder')

        self.output_dir = (
            Path.home() /
            'roboracer_ws' /
            'track_memory'
        )

        self.output_dir.mkdir(
            parents=True,
            exist_ok=True
        )

        self.output_file = (
            self.output_dir /
            'exploration_trajectory.csv'
        )

        # Minimum movement before another point is stored.
        self.record_spacing = 0.05

        self.points = []

        self.last_x = None
        self.last_y = None

        self.finished = False

        self.scan = None
        self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)

        self.pose_sub = self.create_subscription(
            Odometry,
            '/localization/kinematic_state',
            self.pose_callback,
            20
        )

        self.lap_sub = self.create_subscription(
            UInt32,
            '/roboracer/lap_count',
            self.lap_callback,
            10
        )

        self.get_logger().info(
            'Exploration trajectory recorder started'
        )

        self.get_logger().info(
            'Waiting for localization...'
        )

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

    def pose_callback(self, msg):

        if self.finished:
            return

        x = float(
            msg.pose.pose.position.x
        )

        y = float(
            msg.pose.pose.position.y
        )

        yaw = self.quaternion_to_yaw(
            msg.pose.pose.orientation
        )

        # First point.
        if self.last_x is None:

            self.store_point(
                x,
                y,
                yaw
            )

            self.last_x = x
            self.last_y = y

            self.get_logger().info(
                f'RECORDING STARTED: '
                f'x={x:.2f}, y={y:.2f}'
            )

            return

        distance = math.hypot(
            x - self.last_x,
            y - self.last_y
        )

        if distance < self.record_spacing:
            return

        # fix_lap1: skip reversing / re-driving the same bit of track.
        ahead = (
            (x - self.last_x) * math.cos(yaw) +
            (y - self.last_y) * math.sin(yaw)
        )
        if ahead <= 0.0 and distance < 1.0:
            return

        # Reject obviously bad localization jumps.
        if distance > 1.0:

            self.get_logger().warn(
                f'Localization jump ignored: '
                f'{distance:.2f} m'
            )

            self.last_x = x
            self.last_y = y
            return

        self.store_point(
            x,
            y,
            yaw
        )

        self.last_x = x
        self.last_y = y

    def scan_callback(self, msg):
        self.scan = msg

    def store_point(
        self,
        x,
        y,
        yaw
    ):

        # Wall distances to the left / right of the car (lidar, +-10 deg
        # around +-90 deg). Used to build the racing line without a SLAM map.
        if self.scan is not None:
            left = sector_min(self.scan, 80.0, 100.0)
            right = sector_min(self.scan, -100.0, -80.0)
        else:
            left = right = 3.0

        self.points.append(
            [
                len(self.points),
                x,
                y,
                yaw,
                left,
                right
            ]
        )

    def lap_callback(self, msg):

        if self.finished:
            return

        if int(msg.data) < 1:
            return

        self.finished = True

        self.save_trajectory()

        self.get_logger().warn(
            '===================================='
        )

        self.get_logger().warn(
            'EXPLORATION LAP RECORDED'
        )

        self.get_logger().warn(
            f'Points: {len(self.points)}'
        )

        self.get_logger().warn(
            f'File: {self.output_file}'
        )

        self.get_logger().warn(
            '===================================='
        )

    def save_trajectory(self):

        with open(
            self.output_file,
            'w',
            newline=''
        ) as f:

            writer = csv.writer(f)

            writer.writerow(
                [
                    'index',
                    'x',
                    'y',
                    'yaw',
                    'left',
                    'right'
                ]
            )

            for point in self.points:

                writer.writerow(
                    [
                        point[0],
                        f'{point[1]:.6f}',
                        f'{point[2]:.6f}',
                        f'{point[3]:.6f}',
                        f'{point[4]:.3f}',
                        f'{point[5]:.3f}'
                    ]
                )


def main(args=None):

    rclpy.init(args=args)

    node = TrajectoryRecorder()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
