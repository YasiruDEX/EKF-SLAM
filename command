source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch robot_bringup sim_bringup.launch.py

source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch robot_slam slam.launch.py

source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch robot_nav navigation.launch.py

source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 run nav2_map_server map_saver_cli -f ~/aep_maps/classroom_map



source /opt/ros/humble/setup.bash && so
urce install/setup.bash && ros2 launch robot_bringup sim_slam_nav2.launch.py


source /opt/ros/humble/setup.bash && so
urce install/setup.bash && ros2 launch robot_bringup sim_saved_map_nav2.launch.py