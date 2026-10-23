"""Ground-truth odometry for the AutoDRIVE simulator (replaces RF2O)."""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster

SIM = '/autodrive/roboracer_1'


def yaw_from_quat(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def relative_pose(x, y, yaw, x0, y0, yaw0):
    dx, dy = x - x0, y - y0
    c, s = math.cos(-yaw0), math.sin(-yaw0)
    rx = c * dx - s * dy
    ry = s * dx + c * dy
    d = yaw - yaw0
    d = math.atan2(math.sin(d), math.cos(d))
    return rx, ry, d


class SimOdomTf(Node):

    def __init__(self):
        super().__init__('sim_odom_tf')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.start = None

        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        self.tf_pub = TransformBroadcaster(self)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.kin_pub = self.create_publisher(
            Odometry, '/localization/kinematic_state', 10)
        self.create_subscription(Odometry, SIM + '/odom', self.on_odom, qos)
        self.get_logger().info('Sim ground-truth odometry -> TF odom->base_link')

    def on_odom(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = yaw_from_quat(q.x, q.y, q.z, q.w)
        if self.start is None:
            self.start = (p.x, p.y, yaw)
            self.get_logger().info(
                f'Start pose x={p.x:.2f} y={p.y:.2f} yaw={math.degrees(yaw):.1f}')
        x, y, th = relative_pose(p.x, p.y, yaw, *self.start)

        tf = TransformStamped()
        tf.header.stamp = msg.header.stamp
        tf.header.frame_id = self.odom_frame
        tf.child_frame_id = self.base_frame
        tf.transform.translation.x = x
        tf.transform.translation.y = y
        tf.transform.rotation.z = math.sin(th / 2.0)
        tf.transform.rotation.w = math.cos(th / 2.0)
        self.tf_pub.sendTransform(tf)

        o = Odometry()
        o.header.stamp = msg.header.stamp
        o.header.frame_id = self.odom_frame
        o.child_frame_id = self.base_frame
        o.pose.pose.position.x = x
        o.pose.pose.position.y = y
        o.pose.pose.orientation.z = math.sin(th / 2.0)
        o.pose.pose.orientation.w = math.cos(th / 2.0)
        o.twist = msg.twist
        self.odom_pub.publish(o)

        # Pose straight from the simulator, independent of SLAM load.
        k = Odometry()
        k.header.stamp = msg.header.stamp
        k.header.frame_id = 'map'
        k.child_frame_id = self.base_frame
        k.pose.pose = o.pose.pose
        k.twist = msg.twist
        self.kin_pub.publish(k)


def main(args=None):
    rclpy.init(args=args)
    node = SimOdomTf()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
