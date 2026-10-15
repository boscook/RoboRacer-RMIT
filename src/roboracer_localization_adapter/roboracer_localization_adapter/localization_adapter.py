#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration

from nav_msgs.msg import Odometry
from tf2_ros import Buffer, TransformListener, TransformException


def quaternion_multiply(q1, q2):
    """Quaternion multiplication q1 * q2. Format = (x, y, z, w)."""

    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2

    return (
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    )


def rotate_vector(v, q):
    """Rotate vector v using quaternion q."""

    x, y, z = v
    qx, qy, qz, qw = q

    # Quaternion rotation matrix
    r00 = 1.0 - 2.0 * (qy * qy + qz * qz)
    r01 = 2.0 * (qx * qy - qz * qw)
    r02 = 2.0 * (qx * qz + qy * qw)

    r10 = 2.0 * (qx * qy + qz * qw)
    r11 = 1.0 - 2.0 * (qx * qx + qz * qz)
    r12 = 2.0 * (qy * qz - qx * qw)

    r20 = 2.0 * (qx * qz - qy * qw)
    r21 = 2.0 * (qy * qz + qx * qw)
    r22 = 1.0 - 2.0 * (qx * qx + qy * qy)

    return (
        r00 * x + r01 * y + r02 * z,
        r10 * x + r11 * y + r12 * z,
        r20 * x + r21 * y + r22 * z,
    )


class LocalizationAdapter(Node):

    def __init__(self):
        super().__init__('roboracer_localization_adapter')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer,
            self
        )

        self.publisher = self.create_publisher(
            Odometry,
            '/localization/kinematic_state',
            10
        )

        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.last_tf_warning_time = self.get_clock().now()

        self.get_logger().info(
            'RoboRacer localization adapter started'
        )
        self.get_logger().info(
            'Input : /odom'
        )
        self.get_logger().info(
            'Output: /localization/kinematic_state'
        )
        self.get_logger().info(
            'Transforming odom-frame pose into map frame'
        )

    def odom_callback(self, msg):

        try:
            # Use latest available map -> odom transform.
            #
            # SLAM Toolbox normally publishes map -> odom more slowly
            # than RF2O publishes /odom, so using the latest transform
            # prevents unnecessary extrapolation failures.
            transform = self.tf_buffer.lookup_transform(
                'map',
                msg.header.frame_id,
                Time(),
                timeout=Duration(seconds=0.05)
            )

        except TransformException as ex:

            now = self.get_clock().now()

            # Avoid flooding the terminal with warnings.
            if (now - self.last_tf_warning_time).nanoseconds > 2e9:
                self.get_logger().warning(
                    f'Waiting for map -> {msg.header.frame_id}: {ex}'
                )
                self.last_tf_warning_time = now

            return

        # map -> odom translation
        tx = transform.transform.translation.x
        ty = transform.transform.translation.y
        tz = transform.transform.translation.z

        # map -> odom rotation
        q_tf = (
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w
        )

        # odom -> base_link position from RF2O
        p_odom = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        )

        rotated_position = rotate_vector(p_odom, q_tf)

        # odom -> base_link orientation
        q_odom = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        )

        # map -> base_link orientation
        q_map = quaternion_multiply(q_tf, q_odom)

        output = Odometry()

        output.header.stamp = msg.header.stamp
        output.header.frame_id = 'map'
        output.child_frame_id = 'base_link'

        output.pose.pose.position.x = tx + rotated_position[0]
        output.pose.pose.position.y = ty + rotated_position[1]
        output.pose.pose.position.z = tz + rotated_position[2]

        output.pose.pose.orientation.x = q_map[0]
        output.pose.pose.orientation.y = q_map[1]
        output.pose.pose.orientation.z = q_map[2]
        output.pose.pose.orientation.w = q_map[3]

        # RF2O pose covariance.
        # For our planar RoboRacer setup this is retained initially.
        output.pose.covariance = msg.pose.covariance

        # Odometry twist is defined in child_frame_id.
        # RF2O already uses base_link, so it must NOT be rotated into map.
        output.twist = msg.twist

        self.publisher.publish(output)


def main(args=None):

    rclpy.init(args=args)

    node = LocalizationAdapter()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
