from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory("roboracer_bringup"),
        "config",
        "hokuyo_utm10lx.yaml",
    )

    return LaunchDescription([
        Node(
            package="urg_node",
            executable="urg_node_driver",
            name="urg_node_driver",
            output="screen",
            parameters=[config_file],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="laser_static_tf",
            arguments=[
                "--x", "0.20",
                "--y", "0.0",
                "--z", "0.15",
                "--roll", "0.0",
                "--pitch", "0.0",
                "--yaw", "0.0",
                "--frame-id", "base_link",
                "--child-frame-id", "laser",
            ],
            output="screen",
        ),
    ])
