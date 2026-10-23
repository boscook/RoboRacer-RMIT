"""Full RoboRacer stack against the AutoDRIVE simulator (no hardware)."""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

WS = os.path.expanduser('~/roboracer_ws')


def rr(pkg, exe, **kw):
    return Node(package=pkg, executable=exe, name=exe, output='screen', **kw)


def generate_launch_description():
    rf2o_cfg = LaunchConfiguration('rf2o_config')
    slam_cfg = LaunchConfiguration('slam_config')

    return LaunchDescription([
        DeclareLaunchArgument('rf2o_config',
                              default_value=os.path.join(WS, 'config/rf2o.yaml')),
        DeclareLaunchArgument('slam_config',
                              default_value=os.path.join(WS, 'config/slam_toolbox.yaml')),
        DeclareLaunchArgument('max_steer_angle_rad', default_value='0.5236'),
        DeclareLaunchArgument('steer_sign', default_value='1.0'),
        DeclareLaunchArgument('odom_source', default_value='sim'),

        # Sim <-> stack adapter: /scan, base_link->laser TF, actuator commands
        Node(package='roboracer_autodrive_bridge', executable='sim_adapter',
             output='screen',
             parameters=[{
                 'max_steer_angle_rad': LaunchConfiguration('max_steer_angle_rad'),
                 'steer_sign': LaunchConfiguration('steer_sign'),
             }]),

        # Same order and delays as scripts/roboracer_physical.sh
        TimerAction(period=2.0, actions=[
            Node(package='rf2o_laser_odometry',
                 executable='rf2o_laser_odometry_node',
                 name='CLaserOdometry2DNode', output='screen',
                 parameters=[rf2o_cfg],
                 condition=LaunchConfigurationEquals('odom_source', 'rf2o')),
            Node(package='roboracer_autodrive_bridge',
                 executable='sim_odom_tf', output='screen',
                 condition=LaunchConfigurationEquals('odom_source', 'sim'))]),
        TimerAction(period=4.0, actions=[
            Node(package='slam_toolbox', executable='async_slam_toolbox_node',
                 name='slam_toolbox', output='screen',
                 parameters=[slam_cfg, {'use_sim_time': False}])]),
        TimerAction(period=6.0, actions=[
            rr('roboracer_localization_adapter', 'localization_adapter',
               condition=LaunchConfigurationEquals('odom_source', 'rf2o')),
            rr('roboracer_track_detector', 'track_detector'),
            rr('roboracer_track_detector', 'track_controller'),
            rr('roboracer_track_memory', 'lap_detector'),
            rr('roboracer_track_memory', 'trajectory_recorder'),
            rr('roboracer_track_memory', 'memory_manager'),
            rr('roboracer_track_memory', 'trajectory_follower'),
            rr('roboracer_track_memory', 'mode_manager'),
            rr('roboracer_track_memory', 'controller_selector'),
            rr('roboracer_track_memory', 'safety_supervisor'),
            rr('roboracer_track_memory', 'safe_control_adapter'),
        ]),
    ])
