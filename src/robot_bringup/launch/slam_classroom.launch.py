#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, LifecycleNode
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz = LaunchConfiguration('rviz')
    use_gazebo_rviz = LaunchConfiguration('use_gazebo_rviz')

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

    use_gazebo_rviz_arg = DeclareLaunchArgument(
        'use_gazebo_rviz',
        default_value='false',
        description='Launch RViz from robot_gazebo if true'
    )

    bringup_share = get_package_share_directory('robot_bringup')
    gazebo_share = get_package_share_directory('robot_gazebo')
    slam_share = get_package_share_directory('robot_slam')

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_share, 'launch', 'aep_gazebo.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'use_rviz': use_gazebo_rviz
        }.items()
    )

    slam_config = os.path.join(slam_share, 'config', 'slam_toolbox_online_async.yaml')

    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        namespace='',
        output='screen',
        parameters=[
            slam_config,
            {
                'use_sim_time': use_sim_time,
                'map_frame': 'map',
                'odom_frame': 'odom',
                'base_frame': 'AEP_Robot/base_footprint',
                'scan_topic': '/scan',
                'autostart': True, 
                'use_lifecycle_manager': False 
            }
        ],
    )

    odom_to_base_footprint_tf_node = Node(
        package='robot_bringup',
        executable='odom_to_base_footprint_tf.py',
        name='odom_to_base_footprint_tf',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    basefootprint_namespace_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_aep_basefootprint_namespace',
        arguments=[
            '0', '0', '0',
            '0', '0', '0',
            'base_footprint',
            'AEP_Robot/base_footprint'
        ]
    )

    lidar_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_aep_lidar',
        arguments=[
            '0', '0.000420393431200835', '0.32565',
            '0', '0', '0',
            'AEP_Robot/base_footprint',
            'AEP_Robot/base_footprint/lidar'
        ]
    )

    imu_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_aep_imu',
        arguments=[
            '0', '0', '0.15',
            '0', '0', '0',
            'AEP_Robot/base_footprint',
            'AEP_Robot/base_footprint/imu'
        ]
    )

    rviz_config = os.path.join(bringup_share, 'config', 'rviz', 'slam.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
        condition=IfCondition(rviz),
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # Delay SLAM a bit so /clock + /tf exist
    slam_delayed = TimerAction(period=2.0, actions=[slam_toolbox_node])
    rviz_delayed = TimerAction(period=3.0, actions=[rviz_node])

    return LaunchDescription([
        use_sim_time_arg,
        rviz_arg,
        use_gazebo_rviz_arg,
        gazebo_launch,
        odom_to_base_footprint_tf_node,
        basefootprint_namespace_tf,
        lidar_tf,
        imu_tf,
        slam_delayed,
        rviz_delayed,
    ])
