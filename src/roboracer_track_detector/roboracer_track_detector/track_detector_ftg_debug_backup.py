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
        # Follow-The-Gap configuration
        # ==================================================

        # Only use the forward 180 degrees.
        self.fov = math.radians(180.0)

        self.min_range = 0.12
        self.max_range = 6.0

        # Small moving-average filter for noisy LiDAR rays.
        self.smoothing_window = 5

        # Physical/safety bubble around closest obstacle.
        # Car width is ~0.21 m, so this intentionally gives
        # additional lateral clearance.
        self.bubble_radius = 0.25

        # A point needs at least this distance to be considered
        # useful free space.
        self.free_threshold = 0.45

        # Ignore tiny gaps.
        self.minimum_gap_angle = math.radians(10.0)

        # Lookahead target.
        self.lookahead = 1.20

        # Do not ask controller for unrealistic steering.
        self.max_target_angle = math.radians(55.0)

        # Prefer a point with good distance, not necessarily
        # the exact geometric middle of the gap.
        self.best_point_window = math.radians(12.0)

        # Heading smoothing.
        self.filtered_angle = 0.0
        self.alpha = 0.35

        # ==================================================
        # ROS
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
            'RoboRacer Follow-The-Gap detector started'
        )

    # ======================================================
    # Helpers
    # ======================================================

    def smooth_ranges(self, ranges):

        if len(ranges) < self.smoothing_window:
            return ranges[:]

        half = self.smoothing_window // 2
        result = []

        for i in range(len(ranges)):

            start = max(0, i - half)
            end = min(len(ranges), i + half + 1)

            section = ranges[start:end]

            result.append(
                sum(section) / len(section)
            )

        return result

    def create_safety_bubble(
        self,
        ranges,
        angles,
        angle_increment
    ):
        """
        Inflate ALL nearby obstacles by the required vehicle
        clearance instead of bubbling only the closest point.

        This converts the LiDAR scan into approximately the
        free space available to the centre of the car.
        """

        original = ranges[:]

        # Anything closer than this is considered an obstacle
        # worth inflating.
        inflation_distance = 2.0

        for i, distance in enumerate(original):

            if distance <= 0.0:
                continue

            if distance > inflation_distance:
                continue

            # Angular half-width occupied by our safety radius
            # at this obstacle distance.
            effective_distance = max(
                distance,
                self.bubble_radius
            )

            ratio = min(
                1.0,
                self.bubble_radius / effective_distance
            )

            half_angle = math.asin(ratio)

            half_indices = max(
                1,
                int(
                    half_angle /
                    abs(angle_increment)
                )
            )

            start_i = max(
                0,
                i - half_indices
            )

            end_i = min(
                len(ranges) - 1,
                i + half_indices
            )

            # Do NOT blindly zero the region.
            #
            # Propagate the obstacle distance so nearby
            # obstacles become wider in configuration space.
            for j in range(start_i, end_i + 1):
                ranges[j] = 0.0

    def find_gaps(self, ranges, angles):

        gaps = []
        start = None

        for i, distance in enumerate(ranges):

            free = distance >= self.free_threshold

            if free and start is None:
                start = i

            if not free and start is not None:
                gaps.append((start, i - 1))
                start = None

        if start is not None:
            gaps.append((start, len(ranges) - 1))

        valid = []

        for start_i, end_i in gaps:

            width = (
                angles[end_i] -
                angles[start_i]
            )

            if width >= self.minimum_gap_angle:
                valid.append(
                    (start_i, end_i, width)
                )

        return valid

    def select_gap(self, gaps, ranges, angles):

        best_gap = None
        best_score = -1.0

        for start_i, end_i, width in gaps:

            section = ranges[start_i:end_i + 1]

            if not section:
                continue

            mean_distance = (
                sum(section) / len(section)
            )

            max_distance = max(section)

            middle_i = (
                start_i + end_i
            ) // 2

            middle_angle = angles[middle_i]

            # Prefer forward-facing gaps, but still allow
            # significant turns.
            forward = max(
                0.15,
                math.cos(middle_angle)
            )

            score = (
                width *
                (0.60 * mean_distance +
                 0.40 * max_distance) *
                (0.35 + 0.65 * forward)
            )

            if score > best_score:
                best_score = score
                best_gap = (
                    start_i,
                    end_i
                )

        return best_gap

    def select_best_point(
        self,
        start_i,
        end_i,
        ranges,
        angles,
        angle_increment
    ):

        # Smooth again over a larger angular window when
        # selecting the target. This prevents choosing one
        # isolated long LiDAR ray.
        window = max(
            1,
            int(
                self.best_point_window /
                abs(angle_increment)
            )
        )

        best_i = (start_i + end_i) // 2
        best_score = -1.0

        gap_middle = (
            angles[start_i] +
            angles[end_i]
        ) / 2.0

        for i in range(start_i, end_i + 1):

            a = max(start_i, i - window // 2)
            b = min(end_i + 1, i + window // 2 + 1)

            local_distance = (
                sum(ranges[a:b]) /
                max(1, b - a)
            )

            # Prefer deep free space.
            # Slight preference toward gap centre prevents
            # hugging a wall.
            centre_error = abs(
                angles[i] - gap_middle
            )

            centre_preference = max(
                0.25,
                math.cos(centre_error)
            )

            # Also mildly prefer forward travel.
            forward_preference = max(
                0.25,
                math.cos(angles[i])
            )

            score = (
                local_distance *
                (0.55 +
                 0.25 * centre_preference +
                 0.20 * forward_preference)
            )

            if score > best_score:
                best_score = score
                best_i = i

        return best_i

    # ======================================================
    # Main callback
    # ======================================================

    def scan_callback(self, scan):

        angles = []
        ranges = []

        # --------------------------------------------------
        # Extract and sanitize forward scan
        # --------------------------------------------------

        for i, raw in enumerate(scan.ranges):

            angle = (
                scan.angle_min +
                i * scan.angle_increment
            )

            if abs(angle) > self.fov / 2.0:
                continue

            if not math.isfinite(raw):
                distance = self.max_range
            else:
                distance = float(raw)

                if distance < self.min_range:
                    distance = self.min_range

                if distance > self.max_range:
                    distance = self.max_range

            angles.append(angle)
            ranges.append(distance)

        if len(ranges) < 5:
            return

        # --------------------------------------------------
        # Pre-process LiDAR
        # --------------------------------------------------

        processed = self.smooth_ranges(ranges)

        # --------------------------------------------------
        # Safety bubble
        # --------------------------------------------------

        self.create_safety_bubble(
            processed,
            angles,
            scan.angle_increment
        )

        # --------------------------------------------------
        # Find gaps
        # --------------------------------------------------

        gaps = self.find_gaps(
            processed,
            angles
        )

        if not gaps:

            self.get_logger().warn(
                'FTG: no valid gap',
                throttle_duration_sec=1.0
            )

            # No target is published.
            # Controller timeout will command STOP.
            return

        # --------------------------------------------------
        # Select best gap
        # --------------------------------------------------

        best_gap = self.select_gap(
            gaps,
            processed,
            angles
        )

        if best_gap is None:
            return

        start_i, end_i = best_gap

        # --------------------------------------------------
        # Select best point inside gap
        # --------------------------------------------------

        target_i = self.select_best_point(
            start_i,
            end_i,
            processed,
            angles,
            scan.angle_increment
        )

        target_angle = angles[target_i]

        target_angle = max(
            -self.max_target_angle,
            min(
                self.max_target_angle,
                target_angle
            )
        )

        # --------------------------------------------------
        # Smooth heading
        # --------------------------------------------------

        self.filtered_angle = (
            (1.0 - self.alpha) *
            self.filtered_angle +
            self.alpha *
            target_angle
        )

        # --------------------------------------------------
        # Target distance
        # --------------------------------------------------

        target_distance = min(
            self.lookahead,
            max(
                self.free_threshold,
                processed[target_i]
            )
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
        # Target point
        # ==================================================

        target = PointStamped()

        target.header.stamp = scan.header.stamp
        target.header.frame_id = scan.header.frame_id

        target.point.x = target_x
        target.point.y = target_y
        target.point.z = 0.0

        self.target_pub.publish(target)

        # ==================================================
        # Local path
        # ==================================================

        path = Path()
        path.header = target.header

        number_of_points = 20

        for i in range(1, number_of_points + 1):

            ratio = i / number_of_points

            angle = (
                self.filtered_angle *
                ratio
            )

            distance = (
                target_distance *
                ratio
            )

            pose = PoseStamped()

            pose.header = path.header

            pose.pose.position.x = (
                distance *
                math.cos(angle)
            )

            pose.pose.position.y = (
                distance *
                math.sin(angle)
            )

            pose.pose.position.z = 0.0

            half_yaw = angle / 2.0

            pose.pose.orientation.z = (
                math.sin(half_yaw)
            )

            pose.pose.orientation.w = (
                math.cos(half_yaw)
            )

            path.poses.append(pose)

        self.path_pub.publish(path)

        # ==================================================
        # RViz target marker
        # ==================================================

        marker = Marker()

        marker.header = target.header
        marker.ns = 'roboracer_ftg'
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
