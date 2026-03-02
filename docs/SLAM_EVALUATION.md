# SLAM Quantitative Evaluation Documentation

## Overview

This document describes the files and methodology used for quantitative evaluation of the SLAM system using the **EVO** (Evaluation of Odometry and SLAM) tool.

---

## Files Involved

### Main Script

| File | Description |
|------|-------------|
| `evaluate_slam.sh` | Automated pipeline that launches simulation, records data, and runs EVO analysis |

### Helper Nodes

| File | Description |
|------|-------------|
| `src/robot_slam_toolbox/scripts/tf_to_pose.py` | Converts TF transforms (`map→base_link`) to `PoseStamped` messages (`/estimated_pose`) |
| `src/robot_slam_toolbox/scripts/odom_to_pose.py` | Converts Odometry messages to `PoseStamped` (`/ground_truth_pose`) |

### Output Directory

| Directory | Contents |
|-----------|----------|
| `results/` | EVO outputs: plots (PNG), result archives (ZIP) |
| `slam_eval_bag/` | Recorded ROS 2 bag with trajectory data |

---

## Evaluation Methodology

### 1. Data Sources

```
┌─────────────────────────────────────────────────────────────┐
│                    IGNITION GAZEBO                          │
│  ┌──────────────────┐     ┌───────────────────────────────┐ │
│  │  DiffDrive Plugin│────▶│  /odom (Odometry)             │ │
│  │  (Perfect Odom)  │     │  Perfect simulation odometry  │ │
│  └──────────────────┘     └─────────────┬─────────────────┘ │
└─────────────────────────────────────────┼───────────────────┘
                                          │
                                          ▼
┌─────────────────────────────────────────────────────────────┐
│                    ROS 2 NODES                              │
│                                                             │
│  ┌──────────────────┐     ┌───────────────────────────────┐ │
│  │  odom_to_pose.py │────▶│  /ground_truth_pose           │ │
│  │                  │     │  (PoseStamped)                │ │
│  └──────────────────┘     └───────────────────────────────┘ │
│                                                             │
│  ┌──────────────────┐     ┌───────────────────────────────┐ │
│  │  SLAM Toolbox    │────▶│  TF: map → odom → base_link   │ │
│  └──────────────────┘     └─────────────┬─────────────────┘ │
│                                         │                   │
│                                         ▼                   │
│  ┌──────────────────┐     ┌───────────────────────────────┐ │
│  │  tf_to_pose.py   │────▶│  /estimated_pose              │ │
│  │                  │     │  (PoseStamped)                │ │
│  └──────────────────┘     └───────────────────────────────┘ │
└─────────────────────────────────────────────────────────────┘
```

### 2. Pipeline Steps

1. **Launch Simulation**: Starts Ignition Gazebo, RViz, and SLAM Toolbox
2. **Start Helper Nodes**: Runs `tf_to_pose.py` and `odom_to_pose.py`
3. **Record Data**: Uses `ros2 bag record` to capture `/ground_truth_pose` and `/estimated_pose`
4. **Move Robot**: Publishes velocity commands to create a circular trajectory
5. **Run EVO Analysis**: Calculates APE, RPE, and generates trajectory plots

### 3. Metrics Calculated

| Metric | Full Name | Description |
|--------|-----------|-------------|
| **APE** | Absolute Pose Error | Global consistency - error at each point in time |
| **RPE** | Relative Pose Error | Local consistency - drift between consecutive poses |
| **RMSE** | Root Mean Square Error | Summary statistic (lower = better) |

### 4. EVO Commands Used

```bash
# Absolute Pose Error
evo_ape bag2 <bag_file> /ground_truth_pose /estimated_pose

# Relative Pose Error
evo_rpe bag2 <bag_file> /ground_truth_pose /estimated_pose

# Trajectory Visualization
evo_traj bag2 <bag_file> /ground_truth_pose /estimated_pose
```

---

## Usage

```bash
# Run full evaluation
./evaluate_slam.sh

# View results
ls results/
```

---

## Results Interpretation

| APE RMSE | Interpretation |
|----------|----------------|
| < 0.05m  | Excellent |
| 0.05-0.1m | Good |
| 0.1-0.5m | Acceptable |
| > 0.5m   | Poor |

**Latest Results:**
- APE RMSE: **0.024m (2.4cm)** - Excellent
- RPE RMSE: **0.003m (3mm)** - Excellent
