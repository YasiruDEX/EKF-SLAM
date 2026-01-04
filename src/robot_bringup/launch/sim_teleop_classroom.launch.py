#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz = LaunchConfiguration('rviz')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time if true'
    )

    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz if true'
    )

    bringup_share = get_package_share_directory('robot_bringup')
    sim_share = get_package_share_directory('robot_sim')
    desc_share = get_package_share_directory('robot_description')
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')

    urdf_xacro_path = os.path.join(desc_share, 'urdf', 'main.urdf.xacro')
    robot_description = ParameterValue(Command(['xacro ', urdf_xacro_path]), value_type=str)

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

    # Ensure Gazebo can resolve package://robot_description/...
    gz_resource_path = os.path.join(desc_share, '..')
    os.environ['GZ_SIM_RESOURCE_PATH'] = (
        gz_resource_path + ':' + os.environ.get('GZ_SIM_RESOURCE_PATH', '')
        if os.environ.get('GZ_SIM_RESOURCE_PATH') else gz_resource_path
    )

    # Gazebo world
    world_path = os.path.join(sim_share, 'worlds', 'classroom.sdf')
    gz_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': ['-r ', world_path]}.items()
    )

    # Bridges (directional)
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    odom_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='odom_bridge',
        arguments=['/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    scan_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='scan_bridge',
        arguments=['/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    joint_state_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='joint_state_bridge',
        arguments=['/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    cmd_vel_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='cmd_vel_bridge',
        arguments=['/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    odom_to_base_footprint_tf_node = Node(
        package='robot_bringup',
        executable='odom_to_base_footprint_tf.py',
        name='odom_to_base_footprint_tf',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    spawn_robot_node = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_AEP_Robot',
        output='screen',
        arguments=[
            '-world', 'classroom_world',      
            '-topic', 'robot_description',
            '-name', 'AEP_Robot',
            '-x', '2.0',
            '-y', '-3.0',
            '-z', '0.1',
            '-Y', '1.57',
        ]
    )

    spawn_delayed = TimerAction(period=2.0, actions=[spawn_robot_node])

    # RViz
    rviz_config = os.path.join(bringup_share, 'config', 'rviz', 'sim.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
        condition=IfCondition(rviz),
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        use_sim_time_arg,
        rviz_arg,

        gz_server,

        robot_state_publisher_node,

        clock_bridge,
        odom_bridge,
        scan_bridge,
        joint_state_bridge,
        cmd_vel_bridge,
        odom_to_base_footprint_tf_node,

        spawn_delayed,
        rviz_node,
    ])
