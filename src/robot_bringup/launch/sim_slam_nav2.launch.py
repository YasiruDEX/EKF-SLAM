#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true'
    )

    bringup_share = get_package_share_directory('robot_bringup')
    slam_share = get_package_share_directory('robot_slam')
    nav_share = get_package_share_directory('robot_nav')
    rotate_share = get_package_share_directory('robot_rotate_goal')
    firebase_bridge_share = get_package_share_directory('robot_firebase_doa_bridge')

    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, 'launch', 'sim_bringup.launch.py')
        )
    )

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_share, 'launch', 'slam.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items()
    )

    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_share, 'launch', 'navigation.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items()
    )

    rotate_goal_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rotate_share, 'launch', 'rotate_goal_service.launch.py')
        )
    )

    firebase_bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(firebase_bridge_share, 'launch', 'firebase_doa_bridge.launch.py')
        )
    )

    return LaunchDescription([
        use_sim_time_arg,
        sim_launch,
        TimerAction(period=2.0, actions=[slam_launch]),
        TimerAction(period=4.0, actions=[nav_launch]),
        TimerAction(period=6.0, actions=[rotate_goal_launch]),
        TimerAction(period=7.0, actions=[firebase_bridge_launch]),
    ])
