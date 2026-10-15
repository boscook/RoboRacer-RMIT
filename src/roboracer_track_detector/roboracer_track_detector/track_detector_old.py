#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PointStamped, PoseStamped
from visualization_msgs.msg import Marker
from nav_msgs.msg import Path


class TrackDetector(Node):

    def __init__(self):
        super().__init__('roboracer_track_detector')

        # ==================================================
        # Detector configuration
        # ==================================================

        # Search +/- 90 degrees in front of the car.
        self.fov = math.radians(180.0)

        # LiDAR filtering.
        self.min_range = 0.12
        self.max_range = 6.0

        # A ray must have at least this much free distance
        # to belong to a drivable gap.
        self.clearance_threshold = 0.70

        # Ignore extremely narrow openings.
        self.minimum_gap_angle = math.radians(12.0)

        # Local target distance.
        self.lookahead = 1.0

        # Maximum target angle.
        self.max_target_angle = math.radians(55.0)

        # Target smoothing.
        self.filtered_angle = 0.0
        self.alpha = 0.30

        # ==================================================
        # ROS interfaces
        # ==================================================

        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.target_pub = self.create_publisher(
            PointStamped,
            '/roboracer/target_point',
            10
        )

        self.path_pub = self.create_publisher(
            Path,
            '/roboracer/local_path',
            10
        )

        self.marker_pub = self.create_publisher(
            Marker,
            '/roboracer/target_marker',
            10
        )

        self.get_logger().info(
            'RoboRacer gap-based track detector started'
        )

    # ======================================================
    # LaserScan callback
    # ======================================================

    def scan_callback(self, scan):

        rays = []

        # --------------------------------------------------
        # Extract forward scan
        # --------------------------------------------------

        for i, raw in enumerate(scan.ranges):

            angle = scan.angle_min + i * scan.angle_increment

            if abs(angle) > self.fov / 2.0:
                continue

            if not math.isfinite(raw):
                distance = self.max_range
            else:
                distance = max(
                    self.min_range,
                    min(float(raw), self.max_range)
                )

            rays.append((angle, distance))

        if len(rays) < 3:
            return

        # --------------------------------------------------
        # Find contiguous free gaps
        # --------------------------------------------------

        gaps = []
        start = None

        for i, (_, distance) in enumerate(rays):

            free = distance >= self.clearance_threshold

            if free and start is None:
                start = i

            if not free and start is not None:
                gaps.append((start, i - 1))
                start = None

        if start is not None:
            gaps.append((start, len(rays) - 1))

        # --------------------------------------------------
        # Reject tiny gaps
        # --------------------------------------------------

        valid_gaps = []

        for start_i, end_i in gaps:

            start_angle = rays[start_i][0]
            end_angle = rays[end_i][0]

            angular_width = end_angle - start_angle

            if angular_width >= self.minimum_gap_angle:
                valid_gaps.append(
                    (start_i, end_i, angular_width)
                )

        if not valid_gaps:
            self.get_logger().warn(
                'No valid drivable gap detected',
                throttle_duration_sec=1.0
            )
            return

        # --------------------------------------------------
        # Score each gap
        #
        # Prefer:
        #   - wide gaps
        #   - large clearance
        #   - gaps near straight ahead
        # --------------------------------------------------

        best_gap = None
        best_score = -1.0

        for start_i, end_i, width in valid_gaps:

            middle_i = (start_i + end_i) // 2
            middle_angle = rays[middle_i][0]

            distances = [
                rays[j][1]
                for j in range(start_i, end_i + 1)
            ]

            mean_distance = (
                sum(distances) / len(distances)
            )

            forward_preference = max(
                0.0,
                math.cos(middle_angle)
            )

            score = (
                width *
                mean_distance *
                (0.40 + 0.60 * forward_preference)
            )

            if score > best_score:
                best_score = score
                best_gap = (
                    start_i,
                    end_i,
                    middle_angle,
                    mean_distance
                )

        _, _, target_angle, gap_distance = best_gap

        # --------------------------------------------------
        # Limit steering target
        # --------------------------------------------------

        target_angle = max(
            -self.max_target_angle,
            min(self.max_target_angle, target_angle)
        )

        # --------------------------------------------------
        # Smooth target
        # --------------------------------------------------

        self.filtered_angle = (
            (1.0 - self.alpha) * self.filtered_angle +
            self.alpha * target_angle
        )

        # --------------------------------------------------
        # Target distance
        # --------------------------------------------------

        target_distance = min(
            self.lookahead,
            gap_distance
        )

        target_x = (
            target_distance *
            math.cos(self.filtered_angle)
        )

        target_y = (
            target_distance *
            math.sin(self.filtered_angle)
        )

        # ==================================================
        # Publish target point
        # ==================================================

        target = PointStamped()

        # Use EXACT LaserScan timestamp.
        target.header.stamp = scan.header.stamp
        target.header.frame_id = scan.header.frame_id

        target.point.x = target_x
        target.point.y = target_y
        target.point.z = 0.0

        self.target_pub.publish(target)

        # ==================================================
        # Generate curved local path
        # ==================================================

        path = Path()
        path.header = target.header

        number_of_points = 15

        for i in range(1, number_of_points + 1):

            ratio = i / number_of_points

            # Gradually turn toward the selected direction
            # instead of creating one straight diagonal line.
            angle = self.filtered_angle * ratio
            distance = target_distance * ratio

            pose = PoseStamped()
            pose.header = path.header

            pose.pose.position.x = (
                distance * math.cos(angle)
            )

            pose.pose.position.y = (
                distance * math.sin(angle)
            )

            pose.pose.position.z = 0.0

            half_yaw = angle / 2.0

            pose.pose.orientation.z = math.sin(half_yaw)
            pose.pose.orientation.w = math.cos(half_yaw)

            path.poses.append(pose)

        self.path_pub.publish(path)

        # ==================================================
        # Target marker
        # ==================================================

        marker = Marker()

        marker.header = target.header
        marker.ns = 'roboracer_track'
        marker.id = 0

        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position.x = target_x
        marker.pose.position.y = target_y
        marker.pose.position.z = 0.05

        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.12
        marker.scale.y = 0.12
        marker.scale.z = 0.12

        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        self.marker_pub.publish(marker)


def main(args=None):

    rclpy.init(args=args)

    node = TrackDetector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
