#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():

    bringup_share = get_package_share_directory(
        'roboracer_bringup'
    )

    # ============================================================
    # Existing working hardware + SLAM + localization system
    # ============================================================

    full_system = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                bringup_share,
                'launch',
                'full_system_test.launch.py'
            )
        )
    )

    # ============================================================
    # Live track detector
    # Wait for Hokuyo /scan and the rest of the system to start.
    # ============================================================

    track_detector = TimerAction(
        period=12.0,
        actions=[
            Node(
                package='roboracer_track_detector',
                executable='track_detector',
                name='roboracer_track_detector',
                output='screen'
            )
        ]
    )

    # ============================================================
    # MAX autonomous controller
    # Start after detector is already producing targets.
    # ============================================================

    track_controller = TimerAction(
        period=15.0,
        actions=[
            Node(
                package='roboracer_track_detector',
                executable='track_controller_max',
                name='roboracer_track_controller_max',
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        full_system,
        track_detector,
        track_controller,
    ])
