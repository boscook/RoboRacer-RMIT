#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from autoware_control_msgs.msg import Control
from autoware_vehicle_msgs.msg import (
    VelocityReport,
    SteeringReport,
    ControlModeReport,
    GearReport,
)
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64


class VescBridge(Node):

    def __init__(self):
        super().__init__('roboracer_vesc_bridge')

        # --------------------------------------------------
        # RoboRacer actuator calibration / safety limits
        # --------------------------------------------------

        # Measured steering calibration
        # Physical endpoints:
        #   0.28 = maximum RIGHT
        #   0.45 = CENTER
        #   0.72 = maximum LEFT
        #
        # Use slightly reduced limits to avoid loading the mechanical stops.
        self.servo_center = 0.45
        self.servo_min = 0.30
        self.servo_max = 0.70

        # Maximum commanded tire steering angle.
        # Positive Autoware steering angle = LEFT.
        # Negative Autoware steering angle = RIGHT.
        self.max_steering_angle = 0.35

        # Initial physical bring-up limits
        self.max_velocity = 0.30       # m/s
        self.max_duty = 1.00          # 3% maximum duty

        self.timeout = 0.5             # seconds
        self.last_command_time = None

        # Last known states used for Autoware reports
        self.current_steering_angle = 0.0
        self.longitudinal_velocity = 0.0
        self.lateral_velocity = 0.0
        self.heading_rate = 0.0

        # --------------------------------------------------
        # VESC command publishers
        # --------------------------------------------------

        self.servo_pub = self.create_publisher(
            Float64,
            '/commands/servo/position',
            10
        )

        self.motor_pub = self.create_publisher(
            Float64,
            '/commands/motor/duty_cycle',
            10
        )

        # --------------------------------------------------
        # Autoware vehicle-status publishers
        # --------------------------------------------------

        self.velocity_pub = self.create_publisher(
            VelocityReport,
            '/vehicle/status/velocity_status',
            10
        )

        self.steering_pub = self.create_publisher(
            SteeringReport,
            '/vehicle/status/steering_status',
            10
        )

        self.control_mode_pub = self.create_publisher(
            ControlModeReport,
            '/vehicle/status/control_mode',
            10
        )

        self.gear_pub = self.create_publisher(
            GearReport,
            '/vehicle/status/gear_status',
            10
        )

        # --------------------------------------------------
        # Subscriptions
        # --------------------------------------------------

        # Autoware control output
        self.control_sub = self.create_subscription(
            Control,
            '/control/command/control_cmd',
            self.control_callback,
            10
        )

        # RF2O vehicle odometry
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            20
        )

        # --------------------------------------------------
        # Timers
        # --------------------------------------------------

        # Existing command watchdog
        self.watchdog_timer = self.create_timer(
            0.1,
            self.watchdog
        )

        # Publish Autoware vehicle status at 20 Hz
        self.status_timer = self.create_timer(
            0.05,
            self.publish_vehicle_status
        )

        self.get_logger().info(
            'RoboRacer Autoware <-> VESC interface started'
        )
        self.get_logger().info(
            'MAX MOTOR DUTY = 0.03'
        )
        self.get_logger().info(
            'Control input: /control/command/control_cmd'
        )
        self.get_logger().info(
            'Velocity feedback: /odom'
        )
        self.get_logger().info(
            'Publishing Autoware vehicle status at 20 Hz'
        )

    # ==================================================
    # Autoware control command -> VESC
    # ==================================================

    def control_callback(self, msg):

        self.last_command_time = self.get_clock().now()

        # -----------------------
        # Steering
        # -----------------------

        steering_angle = float(
            msg.lateral.steering_tire_angle
        )

        self.current_steering_angle = steering_angle

        # Asymmetric steering calibration:
        #   -0.35 rad -> 0.30 -> RIGHT
        #    0.00 rad -> 0.45 -> CENTER
        #   +0.35 rad -> 0.70 -> LEFT
        steering_angle = max(
            -self.max_steering_angle,
            min(self.max_steering_angle, steering_angle)
        )

        if steering_angle >= 0.0:
            # LEFT
            servo = self.servo_center + (
                steering_angle / self.max_steering_angle
            ) * (self.servo_max - self.servo_center)
        else:
            # RIGHT
            servo = self.servo_center + (
                steering_angle / self.max_steering_angle
            ) * (self.servo_center - self.servo_min)

        servo = max(
            self.servo_min,
            min(self.servo_max, servo)
        )

        servo_msg = Float64()
        servo_msg.data = servo
        self.servo_pub.publish(servo_msg)

        # -----------------------
        # Motor
        # -----------------------

        velocity = float(
            msg.longitudinal.velocity
        )

        velocity = max(
            -self.max_velocity,
            min(self.max_velocity, velocity)
        )

        duty = (
            velocity / self.max_velocity
        ) * self.max_duty

        motor_msg = Float64()
        motor_msg.data = duty
        self.motor_pub.publish(motor_msg)

        self.get_logger().info(
            f'velocity_cmd={velocity:.2f} m/s  '
            f'duty={duty:.3f}  '
            f'steering={math.degrees(steering_angle):.1f} deg  '
            f'servo={servo:.3f}'
        )

    # ==================================================
    # RF2O odometry -> vehicle velocity feedback
    # ==================================================

    def odom_callback(self, msg):

        self.longitudinal_velocity = float(
            msg.twist.twist.linear.x
        )

        self.lateral_velocity = float(
            msg.twist.twist.linear.y
        )

        self.heading_rate = float(
            msg.twist.twist.angular.z
        )

    # ==================================================
    # Publish reports required by Autoware
    # ==================================================

    def publish_vehicle_status(self):

        now = self.get_clock().now().to_msg()

        # -----------------------
        # VelocityReport
        # -----------------------

        velocity_msg = VelocityReport()

        velocity_msg.header.stamp = now
        velocity_msg.header.frame_id = 'base_link'

        velocity_msg.longitudinal_velocity = (
            self.longitudinal_velocity
        )

        velocity_msg.lateral_velocity = (
            self.lateral_velocity
        )

        velocity_msg.heading_rate = (
            self.heading_rate
        )

        self.velocity_pub.publish(
            velocity_msg
        )

        # -----------------------
        # SteeringReport
        # -----------------------

        steering_msg = SteeringReport()

        steering_msg.stamp = now
        steering_msg.steering_tire_angle = (
            self.current_steering_angle
        )

        self.steering_pub.publish(
            steering_msg
        )

        # -----------------------
        # ControlModeReport
        # -----------------------

        control_mode_msg = ControlModeReport()

        control_mode_msg.stamp = now
        control_mode_msg.mode = (
            ControlModeReport.AUTONOMOUS
        )

        self.control_mode_pub.publish(
            control_mode_msg
        )

        # -----------------------
        # GearReport
        # -----------------------

        gear_msg = GearReport()

        gear_msg.stamp = now

        # Initial RoboRacer configuration.
        # Forward operation is treated as DRIVE.
        gear_msg.report = GearReport.DRIVE

        self.gear_pub.publish(
            gear_msg
        )

    # ==================================================
    # Safety watchdog
    # ==================================================

    def watchdog(self):

        if self.last_command_time is None:
            return

        elapsed = (
            self.get_clock().now() -
            self.last_command_time
        ).nanoseconds / 1e9

        if elapsed > self.timeout:

            # Stop motor
            stop = Float64()
            stop.data = 0.0
            self.motor_pub.publish(stop)

            # Center steering
            center = Float64()
            center.data = self.servo_center
            self.servo_pub.publish(center)

            self.current_steering_angle = 0.0
            self.last_command_time = None

            self.get_logger().warn(
                'Control command timeout - MOTOR STOPPED'
            )


def main(args=None):

    rclpy.init(args=args)

    node = VescBridge()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    # Stop motor before shutting down
    stop = Float64()
    stop.data = 0.0
    node.motor_pub.publish(stop)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
