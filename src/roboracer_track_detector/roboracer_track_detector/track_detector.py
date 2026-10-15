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
        # ROBORACER + RACE-CODE HYBRID CONFIGURATION
        # ==================================================

        # Final race code used about 110 deg forward FOV.
        self.fov = math.radians(110.0)

        self.min_range = 0.12
        self.max_range = 10.0

        # Keep our original more permissive threshold initially.
        # Do NOT copy their 1.5 m threshold until track testing.
        self.clearance_threshold = 0.70

        self.minimum_gap_angle = math.radians(12.0)

        self.lookahead = 1.0
        self.max_target_angle = math.radians(55.0)

        # ------------------------------------------
        # Features adapted from race implementation
        # ------------------------------------------

        # Half approximate vehicle width.
        self.vehicle_half_width = 0.15

        # If something occupies the projected car corridor
        # within this distance, mark it as blocked.
        self.corridor_check_distance = 1.50

        # Obstacle inflation.
        self.bubble_radius = 0.45

        # Minimum-filter window:
        # similar idea to their +/-5 beam min filter.
        self.filter_half_window = 5

        # ------------------------------------------
        # Target filtering
        # ------------------------------------------

        self.filtered_angle = 0.0
        self.alpha = 0.45

        # Prefer not only widest gap, but also:
        #   clearance
        #   forward direction
        self.forward_weight = 0.60

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
            'RoboRacer Race-Hybrid LiDAR detector started'
        )

        self.get_logger().info(
            'Features: 110deg FOV + min filter + '
            'vehicle corridor + obstacle bubble + gap scoring'
        )

    # ======================================================
    # Minimum filter
    # ======================================================

    def minimum_filter(self, ranges):

        if len(ranges) < 3:
            return list(ranges)

        result = []

        n = len(ranges)

        for i in range(n):

            start = max(
                0,
                i - self.filter_half_window
            )

            end = min(
                n,
                i + self.filter_half_window + 1
            )

            result.append(
                min(ranges[start:end])
            )

        return result

    # ======================================================
    # Vehicle corridor mask
    # ======================================================

    def apply_vehicle_corridor(
        self,
        rays
    ):

        output = []

        for angle, distance in rays:

            lateral_position = (
                math.sin(angle) *
                distance
            )

            # Adapted from the race-code concept:
            # if obstacle lies directly in the physical
            # corridor occupied by the car, make it blocked.
            if (
                abs(lateral_position) <
                self.vehicle_half_width and
                distance <
                self.corridor_check_distance
            ):

                output.append(
                    (angle, 0.0)
                )

            else:

                output.append(
                    (angle, distance)
                )

        return output

    # ======================================================
    # Safety bubble
    # ======================================================

    def apply_safety_bubble(
        self,
        rays
    ):

        if not rays:
            return rays

        # Find closest real obstacle.
        valid = [
            (i, distance)
            for i, (_, distance)
            in enumerate(rays)
            if distance >= self.min_range
        ]

        if not valid:
            return rays

        closest_index, closest_distance = min(
            valid,
            key=lambda item: item[1]
        )

        if closest_distance <= 0.0:
            return rays

        # Convert physical bubble radius into angular size.
        #
        # This is geometrically cleaner than simply deleting
        # a fixed number of beams.
        ratio = min(
            1.0,
            self.bubble_radius /
            max(
                closest_distance,
                self.bubble_radius
            )
        )

        bubble_angle = math.asin(
            ratio
        )

        center_angle = rays[
            closest_index
        ][0]

        result = []

        for angle, distance in rays:

            delta = abs(
                angle -
                center_angle
            )

            if delta <= bubble_angle:

                result.append(
                    (angle, 0.0)
                )

            else:

                result.append(
                    (angle, distance)
                )

        return result

    # ======================================================
    # LaserScan
    # ======================================================

    def scan_callback(self, scan):

        raw_rays = []

        # ----------------------------------------------
        # 110 degree forward FOV
        # ----------------------------------------------

        half_fov = (
            self.fov /
            2.0
        )

        for i, raw in enumerate(
            scan.ranges
        ):

            angle = (
                scan.angle_min +
                i * scan.angle_increment
            )

            if abs(angle) > half_fov:
                continue

            if not math.isfinite(raw):

                distance = (
                    self.max_range
                )

            else:

                distance = max(
                    self.min_range,
                    min(
                        float(raw),
                        self.max_range
                    )
                )

            raw_rays.append(
                (
                    angle,
                    distance
                )
            )

        if len(raw_rays) < 10:
            return

        # ----------------------------------------------
        # Minimum filter
        # ----------------------------------------------

        raw_distances = [
            distance
            for _, distance
            in raw_rays
        ]

        filtered_distances = (
            self.minimum_filter(
                raw_distances
            )
        )

        rays = [
            (
                raw_rays[i][0],
                filtered_distances[i]
            )
            for i in range(
                len(raw_rays)
            )
        ]

        # ----------------------------------------------
        # Vehicle footprint corridor
        # ----------------------------------------------

        rays = self.apply_vehicle_corridor(
            rays
        )

        # ----------------------------------------------
        # Obstacle inflation / safety bubble
        # ----------------------------------------------

        rays = self.apply_safety_bubble(
            rays
        )

        # ==================================================
        # FIND CONTIGUOUS FREE GAPS
        # ==================================================

        gaps = []
        start = None

        for i, (_, distance) in enumerate(
            rays
        ):

            free = (
                distance >=
                self.clearance_threshold
            )

            if (
                free and
                start is None
            ):

                start = i

            elif (
                not free and
                start is not None
            ):

                gaps.append(
                    (
                        start,
                        i - 1
                    )
                )

                start = None

        if start is not None:

            gaps.append(
                (
                    start,
                    len(rays) - 1
                )
            )

        # ==================================================
        # REJECT NARROW GAPS
        # ==================================================

        valid_gaps = []

        for start_i, end_i in gaps:

            angular_width = (
                rays[end_i][0] -
                rays[start_i][0]
            )

            if (
                angular_width >=
                self.minimum_gap_angle
            ):

                valid_gaps.append(
                    (
                        start_i,
                        end_i,
                        angular_width
                    )
                )

        if not valid_gaps:

            self.get_logger().warn(
                'NO VALID DRIVABLE GAP',
                throttle_duration_sec=1.0
            )

            return

        # ==================================================
        # SELECT BEST GAP
        #
        # Their race code:
        #   largest gap + midpoint
        #
        # Ours:
        #   gap width
        #       × clearance
        #       × forward preference
        #
        # Keep our scoring because it reduces unnecessary
        # side-selection while retaining their preprocessing.
        # ==================================================

        best_gap = None
        best_score = -1.0

        for (
            start_i,
            end_i,
            width
        ) in valid_gaps:

            middle_i = (
                start_i +
                end_i
            ) // 2

            middle_angle = (
                rays[middle_i][0]
            )

            distances = [
                rays[j][1]
                for j in range(
                    start_i,
                    end_i + 1
                )
            ]

            mean_distance = (
                sum(distances) /
                len(distances)
            )

            forward_preference = max(
                0.0,
                math.cos(
                    middle_angle
                )
            )

            direction_factor = (
                (1.0 - self.forward_weight) +
                self.forward_weight *
                forward_preference
            )

            score = (
                width *
                mean_distance *
                direction_factor
            )

            if score > best_score:

                best_score = score

                best_gap = (
                    start_i,
                    end_i,
                    middle_i,
                    middle_angle,
                    mean_distance
                )

        (
            start_i,
            end_i,
            middle_i,
            target_angle,
            gap_distance
        ) = best_gap

        # ==================================================
        # TARGET ANGLE
        # ==================================================

        target_angle = max(
            -self.max_target_angle,
            min(
                self.max_target_angle,
                target_angle
            )
        )

        self.filtered_angle = (
            (1.0 - self.alpha) *
            self.filtered_angle +
            self.alpha *
            target_angle
        )

        # ==================================================
        # TARGET DISTANCE
        # ==================================================

        target_distance = min(
            self.lookahead,
            gap_distance
        )

        target_x = (
            target_distance *
            math.cos(
                self.filtered_angle
            )
        )

        target_y = (
            target_distance *
            math.sin(
                self.filtered_angle
            )
        )

        # ==================================================
        # TARGET POINT
        # ==================================================

        target = PointStamped()

        target.header.stamp = (
            scan.header.stamp
        )

        target.header.frame_id = (
            scan.header.frame_id
        )

        target.point.x = target_x
        target.point.y = target_y
        target.point.z = 0.0

        self.target_pub.publish(
            target
        )

        # ==================================================
        # LOCAL PATH
        # ==================================================

        path = Path()

        path.header = (
            target.header
        )

        number_of_points = 15

        for i in range(
            1,
            number_of_points + 1
        ):

            ratio = (
                i /
                number_of_points
            )

            angle = (
                self.filtered_angle *
                ratio
            )

            distance = (
                target_distance *
                ratio
            )

            pose = PoseStamped()

            pose.header = (
                path.header
            )

            pose.pose.position.x = (
                distance *
                math.cos(angle)
            )

            pose.pose.position.y = (
                distance *
                math.sin(angle)
            )

            pose.pose.orientation.z = (
                math.sin(
                    angle / 2.0
                )
            )

            pose.pose.orientation.w = (
                math.cos(
                    angle / 2.0
                )
            )

            path.poses.append(
                pose
            )

        self.path_pub.publish(
            path
        )

        # ==================================================
        # RVIZ MARKER
        # ==================================================

        marker = Marker()

        marker.header = (
            target.header
        )

        marker.ns = (
            'roboracer_track'
        )

        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position.x = (
            target_x
        )

        marker.pose.position.y = (
            target_y
        )

        marker.pose.position.z = 0.05
        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.12
        marker.scale.y = 0.12
        marker.scale.z = 0.12

        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        self.marker_pub.publish(
            marker
        )


def main(args=None):

    rclpy.init(
        args=args
    )

    node = TrackDetector()

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
