import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    robot_nav_dir = get_package_share_directory('robot_nav')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    # Configuration variables
    nav_params_file = LaunchConfiguration('params_file',
                                        default=os.path.join(robot_nav_dir, 'config', 'nav2_params.yaml'))
    
    # Include base nav2 navigation launch (without its lifecycle manager)
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav_params_file,
            'autostart': 'true',
            'use_lifecycle_mgr': 'true',
            'map_subscribe_transient_local': 'true'
        }.items()
    )
    
    # Delayed navigation launch to allow SLAM and TF to stabilize
    nav2_delayed = TimerAction(period=5.0, actions=[nav2_launch])

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'params_file',
            default_value=nav_params_file,
            description='Full path to the ROS2 parameters file to use for all launched nodes'),
            
        nav2_delayed,
    ])
