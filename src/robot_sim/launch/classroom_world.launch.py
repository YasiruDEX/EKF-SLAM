#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time if true'
    )

    # Package paths
    desc_share = get_package_share_directory('robot_description')
    sim_share = get_package_share_directory('robot_sim')
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')

    # URDF xacro path
    urdf_xacro_path = os.path.join(desc_share, 'urdf', 'main.urdf.xacro')
    
    # Controller YAML path (not used in simulation - kept for reference only)
    # Simulation uses Gazebo DiffDrive plugin, not ros2_control
    controller_yaml_path = os.path.join(desc_share, 'config', 'controllers.yaml')

    # Process URDF from xacro and replace placeholder with actual path at runtime
    import subprocess
    try:
        xacro_result = subprocess.run(
            ['xacro', urdf_xacro_path],
            capture_output=True,
            text=True,
            check=True
        )
        urdf_content = xacro_result.stdout.replace(
            'CONTROLLER_YAML_PATH_PLACEHOLDER',
            controller_yaml_path
        )
        robot_description = ParameterValue(urdf_content, value_type=str)
    except Exception:
        # Fallback: use Command (placeholder won't be replaced but URDF will work)
        robot_description = ParameterValue(
            Command(['xacro ', urdf_xacro_path]),
            value_type=str
        )

    # World file path
    world_path = os.path.join(sim_share, 'worlds', 'classroom.sdf')

    # Robot state publisher for TF
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description
        }]
    )

    # Joint states come from Gazebo via bridge, not joint_state_publisher
    # joint_state_publisher is only for non-sim (description-only) launches

    # Gazebo server
    gz_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': ['-r -s ', world_path]
        }.items()
    )

    # Spawn robot from /robot_description topic
    # Delay spawn to ensure Gazebo is ready and robot_description is published
    spawn_robot_node = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                name='spawn_robot',
                output='screen',
                arguments=[
                    '-topic', '/robot_description',
                    '-name', 'AEP_Robot',
                    '-x', '2.0',
                    '-y', '-3.0',
                    '-z', '0.1',
                    '-Y', '1.57'
                ],
                parameters=[{
                    'use_sim_time': use_sim_time
                }]
            )
        ]
    )

    return LaunchDescription([
        use_sim_time_arg,
        gz_server,
        robot_state_publisher_node,
        spawn_robot_node,
    ])
