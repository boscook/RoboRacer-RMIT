#!/usr/bin/env python3

import math
import rclpy

from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import LaserScan
from autoware_control_msgs.msg import Control


class TrackController(Node):

    NORMAL = 0
    AVOID_LEFT = 1
    AVOID_RIGHT = 2
    EMERGENCY_STOP = 3

    def __init__(self):
        super().__init__('roboracer_track_controller')

        # ==================================================
        # SPEED
        # ==================================================

        self.enable_motor = True

        # Normal autonomous command.
        self.drive_velocity = 0.20

        # Speed while avoiding an obstacle.
        self.avoid_velocity = 0.10

        # ==================================================
        # OBSTACLE DISTANCES
        # ==================================================

        # Begin active avoidance when obstacle enters here.
        self.avoid_distance = 0.70

        # Emergency stop distance.
        self.stop_distance = 0.25

        # Object must move farther than this before
        # avoidance is considered finished.
        self.clear_distance = 0.90

        # ==================================================
        # STEERING
        # ==================================================

        self.max_steering_angle = 0.35

        # Normal track-following response.
        self.steering_gain = 1.35

        # Steering command used during committed avoidance.
        self.avoid_steering = 0.30

        # ==================================================
        # TIMING / STATE
        # ==================================================

        self.target_timeout = 0.5

        # Once LEFT/RIGHT is selected, keep it for at least
        # this long to prevent rapid left-right oscillation.
        self.min_avoid_time = 0.70

        self.state = self.NORMAL

        self.target_angle = 0.0
        self.last_target_time = None

        self.front_distance = float('inf')
        self.left_clearance = float('inf')
        self.right_clearance = float('inf')

        self.avoid_start_time = None

        # ==================================================
        # ROS
        # ==================================================

        self.target_sub = self.create_subscription(
            PointStamped,
            '/roboracer/target_point',
            self.target_callback,
            10
        )

        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.control_pub = self.create_publisher(
            Control,
            '/control/command/control_cmd',
            10
        )

        self.timer = self.create_timer(
            0.05,
            self.control_loop
        )

        self.get_logger().info(
            'Dynamic Obstacle Avoidance V1 started'
        )

    # ==================================================
    # Target from normal track detector
    # ==================================================

    def target_callback(self, msg):

        x = float(msg.point.x)
        y = float(msg.point.y)

        if x <= 0.05:
            return

        self.target_angle = math.atan2(y, x)
        self.last_target_time = self.get_clock().now()

    # ==================================================
    # Helper: robust sector clearance
    # ==================================================

    def sector_clearance(
        self,
        scan,
        angle_min_deg,
        angle_max_deg
    ):

        values = []

        min_angle = math.radians(angle_min_deg)
        max_angle = math.radians(angle_max_deg)

        for i, distance in enumerate(scan.ranges):

            angle = (
                scan.angle_min +
                i * scan.angle_increment
            )

            if angle < min_angle or angle > max_angle:
                continue

            if not math.isfinite(distance):
                continue

            if distance < scan.range_min:
                continue

            if distance > scan.range_max:
                continue

            values.append(float(distance))

        if not values:
            return float('inf')

        # Do not use one single minimum ray.
        # A low percentile is more resistant to LiDAR noise.
        values.sort()

        index = int(0.10 * (len(values) - 1))

        return values[index]

    # ==================================================
    # LiDAR processing
    # ==================================================

    def scan_callback(self, scan):

        # Narrow centre zone:
        # used for emergency collision detection.
        self.front_distance = self.sector_clearance(
            scan,
            -10.0,
            10.0
        )

        # LEFT escape region.
        self.left_clearance = self.sector_clearance(
            scan,
            15.0,
            70.0
        )

        # RIGHT escape region.
        self.right_clearance = self.sector_clearance(
            scan,
            -70.0,
            -15.0
        )

    # ==================================================
    # State helpers
    # ==================================================

    def set_state(self, new_state):

        if new_state == self.state:
            return

        names = {
            self.NORMAL: 'NORMAL',
            self.AVOID_LEFT: 'AVOID LEFT',
            self.AVOID_RIGHT: 'AVOID RIGHT',
            self.EMERGENCY_STOP: 'EMERGENCY STOP'
        }

        old_name = names.get(self.state, 'UNKNOWN')
        new_name = names.get(new_state, 'UNKNOWN')

        self.get_logger().warn(
            f'STATE: {old_name} -> {new_name}'
        )

        self.state = new_state

        if new_state in (
            self.AVOID_LEFT,
            self.AVOID_RIGHT
        ):
            self.avoid_start_time = self.get_clock().now()

    def avoidance_time(self):

        if self.avoid_start_time is None:
            return 0.0

        return (
            self.get_clock().now() -
            self.avoid_start_time
        ).nanoseconds / 1e9

    # ==================================================
    # Control
    # ==================================================

    def control_loop(self):

        msg = Control()

        # --------------------------------------------------
        # Target validity
        # --------------------------------------------------

        target_valid = False

        if self.last_target_time is not None:

            age = (
                self.get_clock().now() -
                self.last_target_time
            ).nanoseconds / 1e9

            target_valid = age <= self.target_timeout

        # --------------------------------------------------
        # Emergency collision check
        # --------------------------------------------------

        if self.front_distance < self.stop_distance:

            self.set_state(self.EMERGENCY_STOP)

        # --------------------------------------------------
        # Recover from emergency stop
        # --------------------------------------------------

        elif self.state == self.EMERGENCY_STOP:

            # Object has moved away enough to choose
            # an escape direction.
            if self.front_distance > self.stop_distance + 0.10:

                if self.left_clearance >= self.right_clearance:
                    self.set_state(self.AVOID_LEFT)
                else:
                    self.set_state(self.AVOID_RIGHT)

        # --------------------------------------------------
        # NORMAL -> obstacle avoidance
        # --------------------------------------------------

        elif self.state == self.NORMAL:

            if self.front_distance < self.avoid_distance:

                # Commit to side with more free space.
                if self.left_clearance >= self.right_clearance:

                    self.set_state(
                        self.AVOID_LEFT
                    )

                else:

                    self.set_state(
                        self.AVOID_RIGHT
                    )

        # --------------------------------------------------
        # Continue committed avoidance
        # --------------------------------------------------

        elif self.state in (
            self.AVOID_LEFT,
            self.AVOID_RIGHT
        ):

            # Don't immediately switch direction.
            if self.avoidance_time() >= self.min_avoid_time:

                # Return to normal only when front has
                # become clearly open again.
                if self.front_distance > self.clear_distance:

                    self.set_state(
                        self.NORMAL
                    )

        # --------------------------------------------------
        # Calculate steering
        # --------------------------------------------------

        if self.state == self.AVOID_LEFT:

            steering = self.avoid_steering

        elif self.state == self.AVOID_RIGHT:

            steering = -self.avoid_steering

        elif target_valid:

            steering = (
                self.steering_gain *
                self.target_angle
            )

        else:

            steering = 0.0

        steering = max(
            -self.max_steering_angle,
            min(
                self.max_steering_angle,
                steering
            )
        )

        # --------------------------------------------------
        # Calculate speed
        # --------------------------------------------------

        if (
            not self.enable_motor or
            not target_valid
        ):

            velocity = 0.0

        elif self.state == self.EMERGENCY_STOP:

            velocity = 0.0

        elif self.state in (
            self.AVOID_LEFT,
            self.AVOID_RIGHT
        ):

            velocity = self.avoid_velocity

        else:

            velocity = self.drive_velocity

        # --------------------------------------------------
        # Publish Autoware Control
        # --------------------------------------------------

        msg.lateral.steering_tire_angle = steering
        msg.longitudinal.velocity = velocity

        self.control_pub.publish(msg)

        # --------------------------------------------------
        # Diagnostic output
        # --------------------------------------------------

        names = {
            self.NORMAL: 'NORMAL',
            self.AVOID_LEFT: 'AVOID_LEFT',
            self.AVOID_RIGHT: 'AVOID_RIGHT',
            self.EMERGENCY_STOP: 'STOP'
        }

        self.get_logger().info(
            f'{names[self.state]} | '
            f'front={self.front_distance:.2f} '
            f'left={self.left_clearance:.2f} '
            f'right={self.right_clearance:.2f} | '
            f'v={velocity:.2f} | '
            f'steer={math.degrees(steering):.1f} deg',
            throttle_duration_sec=0.5
        )


def main(args=None):

    rclpy.init(args=args)

    node = TrackController()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    # Explicit zero command on shutdown.
    stop = Control()

    stop.longitudinal.velocity = 0.0
    stop.lateral.steering_tire_angle = 0.0

    node.control_pub.publish(stop)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
