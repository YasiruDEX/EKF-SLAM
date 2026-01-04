#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz = LaunchConfiguration('rviz')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true'
    )

    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz if true'
    )

    # URDF xacro path
    urdf_xacro_path = PathJoinSubstitution(
        [FindPackageShare('robot_description'), 'urdf', 'main.urdf.xacro']
    )

    # robot_description parameter from xacro
    robot_description = ParameterValue(
        Command(['xacro ', urdf_xacro_path]),
        value_type=str
    )

    # Robot state publisher
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

    # Joint state publisher GUI (for interactive joint control)
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        parameters=[{
            'robot_description': robot_description
        }]
    )

    # RViz config path
    rviz_config = PathJoinSubstitution(
        [FindPackageShare('robot_description'), 'config', 'rviz', 'description.rviz']
    )

    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
        condition=IfCondition(rviz),
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )

    return LaunchDescription([
        use_sim_time_arg,
        rviz_arg,
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node,
    ])

