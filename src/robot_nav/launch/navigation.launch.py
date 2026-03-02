#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetLaunchConfiguration, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def _expand_map_path(context):
    map_value = LaunchConfiguration('map').perform(context)
    if map_value:
        return [SetLaunchConfiguration('map', os.path.expanduser(map_value))]
    return []


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

    has_map = IfCondition(PythonExpression(["'", map_file, "' != ''"]))
    no_map = IfCondition(PythonExpression(["'", map_file, "' == ''"]))

    # Include Nav2 localization launch only when a saved map is provided
    nav2_localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'localization_launch.py')
        ),
        condition=has_map,
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': map_file,
            'params_file': nav2_params_file,
            'autostart': autostart,
        }.items()
    )

    # Include Nav2 navigation launch immediately for SLAM mode (no map argument)
    nav2_navigation_launch_no_map = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        condition=no_map,
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_params_file,
            'autostart': autostart,
        }.items()
    )

    # In localization-only mode, start navigation after localization stack settles
    nav2_navigation_launch_with_map = TimerAction(
        period=4.0,
        condition=has_map,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file': nav2_params_file,
                'autostart': autostart,
            }.items()
        )]
    )

    return LaunchDescription([
        use_sim_time_arg,
        map_arg,
        autostart_arg,
        OpaqueFunction(function=_expand_map_path),
        nav2_localization_launch,
        nav2_navigation_launch_no_map,
        nav2_navigation_launch_with_map,
    ])
