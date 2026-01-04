import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Directories
    robot_bringup_dir = get_package_share_directory('robot_bringup')
    robot_nav_dir = get_package_share_directory('robot_nav')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    rviz_config_file = os.path.join(robot_nav_dir, 'config', 'nav2.rviz')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),

        # Simulation + SLAM (Disable default RViz)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(robot_bringup_dir, 'launch', 'slam_classroom.launch.py')
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'rviz': 'false' 
            }.items()
        ),

        # Navigation (Nav2)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(robot_nav_dir, 'launch', 'navigation_classroom.launch.py')
            ),
            launch_arguments={'use_sim_time': use_sim_time}.items()
        ),
        
        # RViz with Nav2 config
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        )
    ])
