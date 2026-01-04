#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time if true'
    )

    desc_share = get_package_share_directory('robot_description')
    sim_share = get_package_share_directory('robot_sim')
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')

    urdf_xacro_path = os.path.join(
        desc_share,
        'urdf',
        'aep_main.urdf.xacro'
    )

    world_path = os.path.join(sim_share, 'worlds', 'classroom.sdf')

    gz_resource_root = os.path.dirname(desc_share)
    existing_gz_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    existing_ign_path = os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')
    gz_resource_path = f"{gz_resource_root}:{existing_gz_path}" if existing_gz_path else gz_resource_root
    ign_resource_path = f"{gz_resource_root}:{existing_ign_path}" if existing_ign_path else gz_resource_root

    robot_description = ParameterValue(
        Command(['xacro ', urdf_xacro_path]),
        value_type=str,
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description,
        }]
    )

    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_basefootprint_baselink',
        arguments=[
            '0', '0', '0',
            '0', '0', '0',
            'base_footprint',
            'base_link'
        ]
    )

    gz_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': ['-r ', world_path]
        }.items()
    )

    spawn_robot_node = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_AEP_Robot',
        output='screen',
        arguments=[
            '-world', 'classroom_world',
            '-string', Command(['xacro ', urdf_xacro_path]),
            '-name', 'AEP_Robot',
            '-x', '2.0',
            '-y', '-3.0',
            '-z', '0.1',
            '-Y', '1.57',
        ]
    )

    return LaunchDescription([
        use_sim_time_arg,
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', gz_resource_path),
        SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', ign_resource_path),
        gz_server,
        robot_state_publisher_node,
        static_tf_node,
        spawn_robot_node,
    ])
