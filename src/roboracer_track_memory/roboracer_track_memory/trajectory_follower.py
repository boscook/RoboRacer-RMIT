#!/usr/bin/env python3

import csv
import math
from pathlib import Path

import numpy as np

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float64, UInt32


class TrajectoryFollower(Node):

    def __init__(self):
        super().__init__('roboracer_trajectory_follower')

        # ==================================================
        # PARAMETERS
        # ==================================================

        self.declare_parameter(
            'track_file',
            str(
                Path.home() /
                'roboracer_ws' /
                'track_memory' /
                'active_track' /
                'learned_track.csv'
            )
        )

        self.declare_parameter(
            'lookahead_distance',
            0.60
        )

        self.declare_parameter(
            'wheelbase',
            0.32
        )

        self.declare_parameter(
            'max_steering_angle',
            0.35
        )

        self.declare_parameter(
            'test_mode',
            True
        )

        self.track_file = Path(
            self.get_parameter(
                'track_file'
            ).value
        ).expanduser()

        self.lookahead = float(
            self.get_parameter(
                'lookahead_distance'
            ).value
        )

        self.wheelbase = float(
            self.get_parameter(
                'wheelbase'
            ).value
        )

        self.max_steering = float(
            self.get_parameter(
                'max_steering_angle'
            ).value
        )

        self.test_mode = bool(
            self.get_parameter(
                'test_mode'
            ).value
        )

        # ==================================================
        # LOAD TRACK
        # ==================================================

        (
            self.points,
            self.track_speed
        ) = self.load_track(
            self.track_file
        )

        self.number_of_points = len(
            self.points
        )

        self.last_nearest_index = None

        # ==================================================
        # PUBLISHERS
        # ==================================================

        self.steering_pub = self.create_publisher(
            Float64,
            '/roboracer/follower/steering',
            10
        )

        self.speed_pub = self.create_publisher(
            Float64,
            '/roboracer/follower/speed',
            10
        )

        self.target_pub = self.create_publisher(
            PointStamped,
            '/roboracer/follower/target_point',
            10
        )

        self.index_pub = self.create_publisher(
            UInt32,
            '/roboracer/follower/target_index',
            10
        )

        # ==================================================
        # SUBSCRIBER
        # ==================================================

        self.pose_sub = self.create_subscription(
            Odometry,
            '/localization/kinematic_state',
            self.pose_callback,
            20
        )

        self.get_logger().info(
            '===================================='
        )

        self.get_logger().info(
            'Remembered Track Follower started'
        )

        self.get_logger().info(
            f'Track: {self.track_file}'
        )

        self.get_logger().info(
            f'Waypoints: {self.number_of_points}'
        )

        self.get_logger().info(
            f'Lookahead: {self.lookahead:.2f} m'
        )

        self.get_logger().info(
            f'TEST MODE: {self.test_mode}'
        )

        self.get_logger().info(
            '===================================='
        )

    # ==================================================
    # LOAD TRACK
    # ==================================================

    def load_track(self, path):

        if not path.exists():
            raise RuntimeError(
                f'Track file does not exist: {path}'
            )

        points = []
        speeds = []

        with open(path, 'r') as f:

            reader = csv.DictReader(f)

            for row in reader:

                points.append([
                    float(row['x']),
                    float(row['y'])
                ])

                speeds.append(
                    float(row['target_speed'])
                )

        if len(points) < 5:
            raise RuntimeError(
                'Track contains too few waypoints'
            )

        return (
            np.asarray(
                points,
                dtype=np.float64
            ),
            np.asarray(
                speeds,
                dtype=np.float64
            )
        )

    # ==================================================
    # QUATERNION
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

    # ==================================================
    # NEAREST WAYPOINT
    # ==================================================

    def find_nearest(self, x, y):

        position = np.array([
            x,
            y
        ])

        # For initial acquisition search
        # the entire track.
        if self.last_nearest_index is None:

            distance_squared = np.sum(
                (
                    self.points -
                    position
                ) ** 2,
                axis=1
            )

            index = int(
                np.argmin(
                    distance_squared
                )
            )

            self.last_nearest_index = index

            return index

        # After acquisition only search
        # around the previous location.
        #
        # This helps prevent jumping to a
        # nearby but topologically different
        # section of track.
        search_back = 5
        search_forward = 30

        candidates = []

        for offset in range(
            -search_back,
            search_forward + 1
        ):

            index = (
                self.last_nearest_index +
                offset
            ) % self.number_of_points

            candidates.append(
                index
            )

        candidate_points = self.points[
            candidates
        ]

        distance_squared = np.sum(
            (
                candidate_points -
                position
            ) ** 2,
            axis=1
        )

        local_index = int(
            np.argmin(
                distance_squared
            )
        )

        index = candidates[
            local_index
        ]

        self.last_nearest_index = index

        return index

    # ==================================================
    # LOOKAHEAD
    # ==================================================

    def find_target(
        self,
        nearest_index
    ):

        accumulated = 0.0

        index = nearest_index

        for _ in range(
            self.number_of_points
        ):

            next_index = (
                index + 1
            ) % self.number_of_points

            segment = np.linalg.norm(
                self.points[next_index] -
                self.points[index]
            )

            accumulated += segment

            index = next_index

            if accumulated >= self.lookahead:
                return index

        return nearest_index

    # ==================================================
    # PURE PURSUIT
    # ==================================================

    def calculate_steering(
        self,
        x,
        y,
        yaw,
        target
    ):

        dx = (
            target[0] -
            x
        )

        dy = (
            target[1] -
            y
        )

        # Convert target from map frame
        # into vehicle frame.
        local_x = (
            math.cos(yaw) * dx +
            math.sin(yaw) * dy
        )

        local_y = (
            -math.sin(yaw) * dx +
            math.cos(yaw) * dy
        )

        distance_squared = (
            local_x * local_x +
            local_y * local_y
        )

        if distance_squared < 0.0001:
            return 0.0

        # Pure-pursuit curvature.
        curvature = (
            2.0 *
            local_y /
            distance_squared
        )

        steering = math.atan(
            self.wheelbase *
            curvature
        )

        steering = max(
            -self.max_steering,
            min(
                self.max_steering,
                steering
            )
        )

        return steering

    # ==================================================
    # CALLBACK
    # ==================================================

    def pose_callback(
        self,
        msg
    ):

        x = float(
            msg.pose.pose.position.x
        )

        y = float(
            msg.pose.pose.position.y
        )

        yaw = self.quaternion_to_yaw(
            msg.pose.pose.orientation
        )

        nearest = self.find_nearest(
            x,
            y
        )

        target_index = self.find_target(
            nearest
        )

        target = self.points[
            target_index
        ]

        steering = self.calculate_steering(
            x,
            y,
            yaw,
            target
        )

        target_speed = float(
            self.track_speed[
                target_index
            ]
        )

        # ------------------------------------------
        # Debug outputs only.
        # No physical vehicle command yet.
        # ------------------------------------------

        steering_msg = Float64()
        steering_msg.data = steering

        self.steering_pub.publish(
            steering_msg
        )

        speed_msg = Float64()
        speed_msg.data = target_speed

        self.speed_pub.publish(
            speed_msg
        )

        point_msg = PointStamped()

        point_msg.header = msg.header
        point_msg.header.frame_id = 'map'

        point_msg.point.x = float(
            target[0]
        )

        point_msg.point.y = float(
            target[1]
        )

        point_msg.point.z = 0.0

        self.target_pub.publish(
            point_msg
        )

        index_msg = UInt32()
        index_msg.data = target_index

        self.index_pub.publish(
            index_msg
        )


def main(args=None):

    rclpy.init(
        args=args
    )

    node = TrajectoryFollower()

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
