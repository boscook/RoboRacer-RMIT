#!/usr/bin/env python3

import csv
import math
from collections import deque
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
            2.25
        )

        self.declare_parameter(
            'wheelbase',
            0.32
        )

        self.declare_parameter(
            'max_steering_angle',
            0.52
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

        self.pose_hist = deque(maxlen=64)
        self.v_meas = 0.0
        self.declare_parameter('predict_time', 0.20)
        self.predict_time = float(
            self.get_parameter('predict_time').value)
        self.last_steer = 0.0

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

        # The learned track appears only after lap 1, so wait for it.
        self.points = None
        self.track_speed = None
        self.number_of_points = 0
        self.declare_parameter('max_speed', 3.5)
        self.max_speed = float(
            self.get_parameter('max_speed').value
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

            if accumulated >= self.active_lookahead:
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

    # ==================================================
    # LAP-BY-LAP LEARNING
    # ==================================================

    def ilc_init(self):
        n = self.number_of_points
        self.ilc_gain = 0.3
        self.base_points = self.points.copy()
        self.base_speed = self.track_speed.copy()
        t = (np.roll(self.base_points, -1, axis=0) -
             np.roll(self.base_points, 1, axis=0))
        t /= (np.linalg.norm(t, axis=1, keepdims=True) + 1e-9)
        self.normals = np.column_stack([-t[:, 1], t[:, 0]])
        clr = np.full(n, 0.40)
        try:
            with open(self.track_file) as f:
                rows = list(csv.DictReader(f))
            if len(rows) == n and 'clearance' in rows[0]:
                clr = np.array([float(r['clearance']) for r in rows])
        except Exception:
            pass
        self.shift_limit = np.clip(clr - 0.20, 0.0, 0.35)
        self.offset = np.zeros(n)
        self.err_sum = np.zeros(n)
        self.err_cnt = np.zeros(n)
        self.ilc_prev_index = None
        self.ilc_lap = 0

    @staticmethod
    def circ_smooth(v, passes):
        for _ in range(passes):
            v = 0.25 * np.roll(v, 1) + 0.5 * v + 0.25 * np.roll(v, -1)
        return v

    def ilc_record(self, nearest, x, y):
        n = self.number_of_points
        cand = (nearest + np.arange(-25, 4)) % n
        d2 = np.sum((self.base_points[cand] - [x, y]) ** 2, axis=1)
        nearest = int(cand[int(np.argmin(d2))])
        e = float(np.dot(
            np.array([x, y]) - self.base_points[nearest],
            self.normals[nearest]))
        self.err_sum[nearest] += e
        self.err_cnt[nearest] += 1
        n = self.number_of_points
        prev = self.ilc_prev_index
        if prev is not None and prev > 0.9 * n and nearest < 0.1 * n:
            self.ilc_update()
        self.ilc_prev_index = nearest

    def ilc_update(self):
        n = self.number_of_points
        seen = self.err_cnt > 0
        if seen.sum() < 0.5 * n:
            self.err_sum[:] = 0.0
            self.err_cnt[:] = 0.0
            return
        self.ilc_lap += 1
        e = np.where(seen, self.err_sum / np.maximum(self.err_cnt, 1), 0.0)
        e = self.circ_smooth(e, 3)

        # 1. shift the line against the error, inside the corridor
        self.offset = self.circ_smooth(
            self.offset - self.ilc_gain * e, 2)
        self.offset = np.clip(self.offset, -self.shift_limit,
                              self.shift_limit)
        self.points = self.base_points + self.offset[:, None] * self.normals

        # 2. slow down before corners that were missed, recover elsewhere
        factor = np.ones(n)
        missed = np.where(np.abs(e) > 0.12)[0]
        for i in missed:
            for j in range(16):
                factor[(i - j) % n] = 0.92
        # fix_lap1: never more than 25 % below the base plan.
        self.track_speed = np.maximum(
            np.maximum(0.4, 0.75 * self.base_speed),
            self.track_speed * factor)
        clean = (np.abs(e) < 0.05) & (factor >= 1.0)
        self.track_speed = np.where(
            clean,
            np.minimum(self.base_speed, self.track_speed * 1.02),
            self.track_speed)

        worst = int(np.argmax(np.abs(e)))
        self.get_logger().warn(
            f'LAP LEARNED #{self.ilc_lap}: mean_err={np.mean(np.abs(e)):.3f} m '
            f'max_err={abs(e[worst]):.3f} m at point {worst} | '
            f'slowed {len(missed)} pts | '
            f'max_shift={np.max(np.abs(self.offset)):.2f} m')

        # 3. save for inspection
        try:
            out = self.track_file.parent / 'learned_track_iter.csv'
            with open(out, 'w', newline='') as f:
                w = csv.writer(f)
                w.writerow(['index', 'x', 'y', 'offset', 'error',
                            'target_speed'])
                for i in range(n):
                    w.writerow([i, f'{self.points[i, 0]:.4f}',
                                f'{self.points[i, 1]:.4f}',
                                f'{self.offset[i]:.4f}', f'{e[i]:.4f}',
                                f'{self.track_speed[i]:.3f}'])
        except Exception as ex:
            self.get_logger().warn(f'Could not save learned line: {ex}')

        self.err_sum[:] = 0.0
        self.err_cnt[:] = 0.0

    def try_load(self):

        try:
            if (
                self.track_file.exists() and
                self.track_file.stat().st_size > 50
            ):
                (
                    self.points,
                    self.track_speed
                ) = self.load_track(
                    self.track_file
                )
                self.number_of_points = len(self.points)
                self.last_nearest_index = None
                self.ilc_init()
                self.get_logger().warn(
                    f'LEARNED TRACK LOADED: '
                    f'{self.number_of_points} waypoints'
                )
        except Exception as e:
            self.points = None
            self.get_logger().error(f'Track load failed: {e}')

        return self.points is not None

    def pose_callback(
        self,
        msg
    ):

        if self.points is None and not self.try_load():
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

        # predict_time: steer for where the car will be.
        ax, ay = x, y
        if self.v_meas > 0.3:
            tp = self.predict_time
            yr = self.v_meas * math.tan(self.last_steer) / self.wheelbase
            ym = yaw + 0.5 * yr * tp
            x += self.v_meas * tp * math.cos(ym)
            y += self.v_meas * tp * math.sin(ym)
            yaw += yr * tp

        nearest = self.find_nearest(
            x,
            y
        )

        # Speed-adaptive lookahead (adaptive_lookahead):
        # short in slow corners, up to the parameter on straights.
        self.ilc_record(nearest, ax, ay)

        # turn_in_v2: lookahead from the faster of plan and real speed.
        t_now = self.get_clock().now().nanoseconds / 1e9
        self.pose_hist.append((t_now, ax, ay))
        while (len(self.pose_hist) > 2 and
               t_now - self.pose_hist[1][0] >= 0.3):
            self.pose_hist.popleft()
        t0, x0, y0 = self.pose_hist[0]
        if t_now - t0 > 0.15:
            self.v_meas = math.hypot(ax - x0, ay - y0) / (t_now - t0)
        v_here = max(float(self.track_speed[nearest]), self.v_meas)
        # lookahead_v3: longer at speed (stops weaving above 3 m/s).
        self.active_lookahead = max(
            0.70,
            min(self.lookahead, 0.50 + 0.50 * v_here)
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

        target_speed = min(
            float(
                self.track_speed[
                    target_index
                ]
            ),
            self.max_speed
        )

        # ------------------------------------------
        # Debug outputs only.
        # No physical vehicle command yet.
        # ------------------------------------------

        steering_msg = Float64()
        self.last_steer = steering
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
