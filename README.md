# Autonomous Exam Proctoring Robot (AEPR) - Simulation

This repository contains the simulation and control software for the Autonomous Exam Proctoring Robot (AEPR). The project is built on **ROS 2 Humble** and **Gazebo (Ignition)**, featuring a custom-built Extended Kalman Filter (EKF) SLAM system.

## Project Overview

The AEPR simulation environment provides a comprehensive setup for developing and testing autonomous behaviors. It includes:
*   A custom differential drive robot model (URDF/Xacro).
*   A simulated classroom environment (`simple_test.sdf`).
*   **Custom EKF-SLAM**: A from-scratch Python implementation of Simultaneous Localization and Mapping.
*   Sensor simulation (Lidar, Odometry, IMU).

## Key Features

### Custom EKF-SLAM Implementation
Instead of relying on standard packages like `slam_toolbox`, this project implements a custom SLAM solution to demonstrate core robotics concepts.

*   **Algorithm**: Extended Kalman Filter (EKF) with a feature-based approach.
*   **State Estimation**: Fuses wheel odometry (prediction) with Lidar feature extraction (correction).
*   **Mapping**: Generates a 2D occupancy grid map (`/map`) in real-time.
*   **Scan Matching**: Enhances localization accuracy by aligning laser scans.
*   **Codebase**: Pure Python implementation located in `src/robot_slam`.

### Tech Stack
*   **Middleware**: ROS 2 Humble
*   **Simulation**: Gazebo Fortress (Ignition)
*   **Language**: Python 3, C++
*   **Build System**: colcon

## Installation

### Prerequisites
*   Ubuntu 22.04 LTS
*   ROS 2 Humble
*   Gazebo Fortress

### Build Instructions

1.  **Clone the repository:**
    ```bash
    cd ~/ros2_ws/src
    git clone https://github.com/your-username/autonomous_exam_proctoring_robot.git
    cd ..
    ```

2.  **Install dependencies:**
    ```bash
    rosdep update
    rosdep install --from-paths src --ignore-src -r -y
    ```

3.  **Build the workspace:**
    ```bash
    colcon build
    ```

4.  **Source the setup file:**
    ```bash
    source install/setup.bash
    ```

## Usage

### Launching the Simulation
To start the entire system (Gazebo, Robot Spawn, EKF-SLAM, RViz):

```bash
ros2 launch robot_bringup sim_bringup.launch.py
```

### Teleoperation
To drive the robot manually:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### Visualization
*   **RViz2**: Automatically launches with the simulation. Displays the robot model, laser scan, and the live-generated occupancy map.
*   **TF Tree**: Verified valid `map -> odom -> base_link` TF chain with correct simulation timestamps.

## Package Structure

*   `robot_bringup`: Launch files for the entire simulation system.
*   `robot_description`: URDF/Xacro models and RViz configurations.
*   `robot_sim`: Gazebo world files and simulation resources.
*   `robot_slam`: Custom EKF-SLAM implementation scripts and parameters.

## License
Apache-2.0
