source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch robot_bringup sim_bringup.launch.py

source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch robot_slam slam.launch.py

source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch robot_nav navigation.launch.py