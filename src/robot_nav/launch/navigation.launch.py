#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_file = LaunchConfiguration('map')
    autostart = LaunchConfiguration('autostart')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time if true'
    )

    map_arg = DeclareLaunchArgument(
        'map',
        default_value='',
        description='Full path to map yaml file. Leave empty if using SLAM.'
    )

    autostart_arg = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically start Nav2 nodes'
    )

    # Get paths
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    robot_nav_dir = get_package_share_directory('robot_nav')
    nav2_params_file = os.path.join(robot_nav_dir, 'config', 'nav2_params.yaml')

    # Include Nav2 bringup launch
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_params_file,
            'autostart': autostart,
        }.items()
    )

    return LaunchDescription([
        use_sim_time_arg,
        map_arg,
        autostart_arg,
        nav2_bringup_launch,
    ])
