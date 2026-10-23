import math, time, rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from autoware_control_msgs.msg import Control

class L(Node):
    def __init__(s):
        super().__init__('logger')
        s.t0 = time.time(); s.front = s.left = s.right = float('nan')
        s.spd = s.cv = s.cs = float('nan')
        s.create_subscription(LaserScan, '/scan', s.scan, qos_profile_sensor_data)
        s.create_subscription(Odometry, '/autodrive/roboracer_1/odom', s.odom, qos_profile_sensor_data)
        s.create_subscription(Control, '/control/command/control_cmd', s.cmd, 10)
        s.create_timer(0.5, s.tick)
    def sector(s, m, lo, hi):
        a = m.angle_min; v = []
        for r in m.ranges:
            d = math.degrees(a); a += m.angle_increment
            if lo <= d <= hi and math.isfinite(r) and r > 0.05: v.append(r)
        return min(v) if v else float('inf')
    def scan(s, m):
        s.front = s.sector(m, -10, 10)
        s.left = s.sector(m, 30, 90)
        s.right = s.sector(m, -90, -30)
    def odom(s, m):
        s.spd = math.hypot(m.twist.twist.linear.x, m.twist.twist.linear.y)
    def cmd(s, m):
        lon = m.longitudinal
        s.cv = getattr(lon, 'velocity', getattr(lon, 'speed', float('nan')))
        s.cs = m.lateral.steering_tire_angle
    def tick(s):
        print('t=%5.1f front=%.2f left=%.2f right=%.2f | cmd_v=%.2f cmd_steer=%.3f | sim_v=%.2f' %
              (time.time()-s.t0, s.front, s.left, s.right, s.cv, s.cs, s.spd), flush=True)

rclpy.init(); rclpy.spin(L())
