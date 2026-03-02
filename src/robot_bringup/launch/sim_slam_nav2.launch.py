#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    bringup_share = get_package_share_directory('robot_bringup')
    slam_share = get_package_share_directory('robot_slam')
    nav_share = get_package_share_directory('robot_nav')

    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, 'launch', 'sim_bringup.launch.py')
        )
    )

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_share, 'launch', 'slam.launch.py')
        )
    )

    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_share, 'launch', 'navigation.launch.py')
        )
    )

    return LaunchDescription([
        sim_launch,
        TimerAction(period=2.0, actions=[slam_launch]),
        TimerAction(period=4.0, actions=[nav_launch]),
    ])
