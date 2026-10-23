"""
Adapter between the RoboRacer stack and the AutoDRIVE RoboRacer simulator.

Replaces everything the physical car gets from hardware:
  Hokuyo driver   -> /autodrive/roboracer_1/lidar  re-published as /scan
  base_link->laser static TF (matches the simulated LiDAR mounting)
  VESC bridge     -> /control/command/control_cmd  translated to
                     /autodrive/roboracer_1/{throttle,steering}_command

The simulator's own TF tree (world -> roboracer_1 -> lidar ...) is left
untouched. The RoboRacer stack lives in its own tree
(map -> odom -> base_link -> laser), so the two never share a frame.
"""

import math
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from autoware_control_msgs.msg import Control
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from tf2_ros import StaticTransformBroadcaster

SIM = '/autodrive/roboracer_1'


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


class SpeedController:
    """Feedforward + PI. The simulator takes a normalised throttle, not m/s."""

    def __init__(self, kff, kp, ki, i_limit=3.0):
        self.kff, self.kp, self.ki, self.i_limit = kff, kp, ki, i_limit
        self.integral = 0.0

    def reset(self):
        self.integral = 0.0

    def step(self, v_ref, v_meas, dt):
        err = v_ref - v_meas
        self.integral = clamp(self.integral + err * dt,
                              -self.i_limit, self.i_limit)
        u = self.kff * v_ref + self.kp * err + self.ki * self.integral
        return clamp(u, -1.0, 1.0)


def steering_to_normalised(angle_rad, max_angle_rad, sign):
    return clamp(sign * angle_rad / max_angle_rad, -1.0, 1.0)


class SimAdapter(Node):

    def __init__(self):
        super().__init__('roboracer_sim_adapter')

        # --- parameters (calibrate steer + speed, see README) ---
        self.declare_parameter('max_steer_angle_rad', 0.35)
        self.declare_parameter('steer_sign', 1.0)
        self.declare_parameter('kff', 0.04)
        self.declare_parameter('kp', 0.015)
        self.declare_parameter('ki', 0.03)
        self.declare_parameter('cmd_timeout', 0.5)
        self.declare_parameter('brake_max', 0.0)  # clean_lap1: off
        self.declare_parameter('brake_gain', 0.50)
        self.declare_parameter('laser_frame', 'laser')
        self.declare_parameter('base_frame', 'base_link')
        # Sim vehicle frame is the rear-axle centre; lidar is at
        # (0.2733, 0, 0.096) in that frame (autodrive_bridge.py).
        self.declare_parameter('laser_x', 0.2733)
        self.declare_parameter('laser_y', 0.0)
        self.declare_parameter('laser_z', 0.096)

        self.max_steer = float(self.get_parameter('max_steer_angle_rad').value)
        self.steer_sign = float(self.get_parameter('steer_sign').value)
        self.timeout = float(self.get_parameter('cmd_timeout').value)
        self.brake_max = float(self.get_parameter('brake_max').value)
        self.brake_gain = float(self.get_parameter('brake_gain').value)
        self.v_fwd = 0.0
        self.laser_frame = self.get_parameter('laser_frame').value
        self.pid = SpeedController(
            float(self.get_parameter('kff').value),
            float(self.get_parameter('kp').value),
            float(self.get_parameter('ki').value))

        # Match the simulator bridge QoS (reliable, depth 1).
        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST, depth=1)

        # --- sensors -> stack ---
        self.scan_pub = self.create_publisher(LaserScan, '/scan', qos)
        self.create_subscription(LaserScan, SIM + '/lidar',
                                 self.on_lidar, qos)
        self.create_subscription(Odometry, SIM + '/odom',
                                 self.on_sim_odom, qos)

        # --- stack -> actuators ---
        self.throttle_pub = self.create_publisher(
            Float32, SIM + '/throttle_command', qos)
        self.steer_pub = self.create_publisher(
            Float32, SIM + '/steering_command', qos)
        self.create_subscription(Control, '/control/command/control_cmd',
                                 self.on_control, 10)

        # --- static TF base_link -> laser ---
        self.tf_static = StaticTransformBroadcaster(self)
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = self.get_parameter('base_frame').value
        tf.child_frame_id = self.laser_frame
        tf.transform.translation.x = float(self.get_parameter('laser_x').value)
        tf.transform.translation.y = float(self.get_parameter('laser_y').value)
        tf.transform.translation.z = float(self.get_parameter('laser_z').value)
        tf.transform.rotation.w = 1.0
        self.tf_static.sendTransform(tf)

        self.v_ref = 0.0
        self.v_meas = 0.0
        self.last_u = 0.0
        self.pos_hist = deque(maxlen=64)
        self.steer_cmd = 0.0
        self.last_cmd = None
        self.last_t = self.get_clock().now()
        self.create_timer(0.02, self.actuate)  # 50 Hz

        self.get_logger().info(
            f'Sim adapter up: max_steer={self.max_steer} rad, '
            f'sign={self.steer_sign}')

    # ---------------- sensors ----------------
    def on_lidar(self, msg):
        out = LaserScan()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.laser_frame
        out.angle_min = msg.angle_min
        out.angle_increment = msg.angle_increment
        out.time_increment = msg.time_increment
        out.scan_time = msg.scan_time
        out.range_min = msg.range_min
        out.range_max = msg.range_max
        out.ranges = [float('inf') if math.isnan(r) else float(r)
                      for r in msg.ranges]
        out.intensities = list(msg.intensities)
        # slam_toolbox / RF2O expect angle_max to match the beam count.
        n = len(out.ranges)
        out.angle_max = msg.angle_min + (n - 1) * msg.angle_increment
        self.scan_pub.publish(out)

    def on_sim_odom(self, msg):
        # Odom twist is noisy: use position differencing over >= 0.3 s.
        t = self.get_clock().now().nanoseconds * 1e-9
        pz = msg.pose.pose.position
        self.pos_hist.append((t, pz.x, pz.y))
        while len(self.pos_hist) > 2 and t - self.pos_hist[1][0] >= 0.3:
            self.pos_hist.popleft()
        t0, x0, y0 = self.pos_hist[0]
        if t - t0 > 0.15:
            self.v_meas = math.hypot(pz.x - x0, pz.y - y0) / (t - t0)
            q = msg.pose.pose.orientation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                             1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            self.v_fwd = ((pz.x - x0) * math.cos(yaw) +
                          (pz.y - y0) * math.sin(yaw)) / (t - t0)

    # ---------------- actuation ----------------
    def on_control(self, msg):
        self.last_cmd = self.get_clock().now()
        self.v_ref = float(msg.longitudinal.velocity)
        self.steer_cmd = steering_to_normalised(
            float(msg.lateral.steering_tire_angle),
            self.max_steer, self.steer_sign)

    def actuate(self):
        now = self.get_clock().now()
        dt = (now - self.last_t).nanoseconds * 1e-9
        self.last_t = now

        stale = (self.last_cmd is None or
                 (now - self.last_cmd).nanoseconds * 1e-9 > self.timeout)
        if stale:
            # The sim bridge latches the last command, so zero explicitly.
            self.pid.reset()
            throttle, steer = 0.0, 0.0
        else:
            sign = 1.0 if self.v_ref >= 0.0 else -1.0
            throttle = self.pid.step(self.v_ref, sign * self.v_meas, dt)
            if self.v_ref >= 0.0:
                # The sim reverses on negative throttle: never brake with it.
                throttle = max(0.0, throttle)
            if abs(self.v_ref) < 0.05:
                self.pid.reset()
                throttle = 0.0
            steer = self.steer_cmd

            # Brake only while clearly moving forward too fast.
            over = self.v_fwd - max(0.0, self.v_ref)
            # fix_lap1: no braking below 0.8 m/s, full strength from 1.3.
            if self.v_ref >= 0.0 and self.v_fwd > 0.8 and over > 0.15:
                fade = min(1.0, (self.v_fwd - 0.8) / 0.5)
                throttle = -fade * min(self.brake_max,
                                       self.brake_gain * over)
                self.pid.integral = max(0.0, self.pid.integral)

        # Limit how fast throttle can RISE; cutting power stays instant.
        throttle = min(throttle, self.last_u + 1.5 * dt)
        self.last_u = throttle

        t, s = Float32(), Float32()
        t.data, s.data = float(throttle), float(steer)
        self.throttle_pub.publish(t)
        self.steer_pub.publish(s)


def main():
    rclpy.init()
    node = SimAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        z = Float32()
        z.data = 0.0
        node.throttle_pub.publish(z)
        node.steer_pub.publish(z)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
