from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    sensor_launch = os.path.join(
        get_package_share_directory('roboracer_sensor_kit_launch'),
        'launch',
        'sensing.launch.xml'
    )

    vehicle_launch = os.path.join(
        get_package_share_directory('roboracer_vehicle_launch'),
        'launch',
        'vehicle_interface.launch.xml'
    )

    slam_config = os.path.expanduser(
        '~/roboracer_ws/config/slam_toolbox.yaml'
    )

    rf2o_config = os.path.expanduser(
        '~/roboracer_ws/config/rf2o.yaml'
    )

    return LaunchDescription([

        # Hokuyo + LaserScan -> PointCloud2
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(sensor_launch),
            launch_arguments={
                'launch_driver': 'true',
                'vehicle_mirror_param_file': '',
            }.items()
        ),

        # VESC + Autoware vehicle bridge
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(vehicle_launch)
        ),

        # Static base_link -> laser TF
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='laser_static_tf',
            arguments=[
                '--x', '0.20',
                '--y', '0.0',
                '--z', '0.15',
                '--roll', '0.0',
                '--pitch', '0.0',
                '--yaw', '0.0',
                '--frame-id', 'base_link',
                '--child-frame-id', 'laser'
            ],
            output='screen'
        ),

        # Delay RF2O so /scan and base_link -> laser TF are ready first
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='rf2o_laser_odometry',
                    executable='rf2o_laser_odometry_node',
                    name='rf2o_laser_odometry',
                    parameters=[rf2o_config],
                    output='screen'
                )
            ]
        ),

        # Delay SLAM slightly after RF2O
        TimerAction(
            period=7.0,
            actions=[
                Node(
                    package='slam_toolbox',
                    executable='async_slam_toolbox_node',
                    name='slam_toolbox',
                    parameters=[
                        slam_config,
                        {'use_sim_time': False}
                    ],
                    output='screen'
                )
            ]
        ),

        # Delay localization adapter until odom + map TF are available
        TimerAction(
            period=9.0,
            actions=[
                Node(
                    package='roboracer_localization_adapter',
                    executable='localization_adapter',
                    name='roboracer_localization_adapter',
                    output='screen'
                )
            ]
        ),
    ])
