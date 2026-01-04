# 2. Methodology

This section presents the theoretical foundation and implementation details of our custom Extended Kalman Filter (EKF) based Simultaneous Localization and Mapping (SLAM) system for the autonomous exam proctoring robot.

## 2.1 Problem Formulation

The SLAM problem addresses the joint estimation of the robot's pose and the environment map from noisy sensor observations. Given:
- A sequence of control inputs (velocity commands)
- A sequence of sensor observations (laser scans)

The objective is to estimate:
- The robot's trajectory through the environment
- A consistent map representation of the environment

Our implementation decouples the full SLAM problem into:
1. **Localization**: EKF-based pose estimation using a velocity motion model
2. **Mapping**: Occupancy grid construction using log-odds probability updates

This approach maintains computational efficiency while providing real-time performance suitable for mobile robot navigation.

## 2.2 State Vector and Measurement Definition

### State Vector

The robot state is represented as a 3-dimensional vector:

$$\mathbf{x}_t = \begin{bmatrix} x \\ y \\ \theta \end{bmatrix}$$

Where:
- $x$: Robot position along the x-axis (meters)
- $y$: Robot position along the y-axis (meters)  
- $\theta$: Robot orientation/heading angle (radians, normalized to $[-\pi, \pi]$)

### State Covariance

The uncertainty in the state estimate is represented by a $3 \times 3$ covariance matrix:

$$\mathbf{P}_t = \begin{bmatrix} 
\sigma_{xx}^2 & \sigma_{xy} & \sigma_{x\theta} \\
\sigma_{xy} & \sigma_{yy}^2 & \sigma_{y\theta} \\
\sigma_{x\theta} & \sigma_{y\theta} & \sigma_{\theta\theta}^2
\end{bmatrix}$$

The covariance matrix is initialized with small values ($\sigma^2 = 0.01$) reflecting initial certainty about the robot's starting pose.

### Measurements

The system processes two types of measurements:

1. **Odometry measurements**: Linear velocity $v$ (m/s) and angular velocity $\omega$ (rad/s) from wheel encoders
2. **Laser scan measurements**: Range and bearing data from a 2D LiDAR sensor used for scan matching and occupancy grid updates

## 2.3 Motion Model (Prediction)

The prediction step uses a velocity-based motion model that accounts for both straight-line and arc motion.

### Velocity Motion Model

For angular velocity $|\omega| < 10^{-6}$ (straight-line motion):

$$\mathbf{x}_{t+1} = \begin{bmatrix}
x_t + v \cdot \cos(\theta_t) \cdot \Delta t \\
y_t + v \cdot \sin(\theta_t) \cdot \Delta t \\
\theta_t
\end{bmatrix}$$

For $|\omega| \geq 10^{-6}$ (arc motion):

$$\mathbf{x}_{t+1} = \begin{bmatrix}
x_t - \frac{v}{\omega} \sin(\theta_t) + \frac{v}{\omega} \sin(\theta_t + \omega \cdot \Delta t) \\
y_t + \frac{v}{\omega} \cos(\theta_t) - \frac{v}{\omega} \cos(\theta_t + \omega \cdot \Delta t) \\
\theta_t + \omega \cdot \Delta t
\end{bmatrix}$$

Where $\Delta t$ is the time step between consecutive odometry updates.

## 2.4 Measurement Model (Update)

The update step incorporates two measurement sources:

### Odometry-Based Update

When using odometry as a direct pose observation, the measurement model is:

$$\mathbf{z}_t = \mathbf{H} \cdot \mathbf{x}_t + \mathbf{n}_t$$

Where $\mathbf{H} = \mathbf{I}_{3 \times 3}$ (identity matrix) and $\mathbf{n}_t \sim \mathcal{N}(0, \mathbf{Q}_{odom})$.

Default odometry measurement covariance:
$$\mathbf{Q}_{odom} = \text{diag}(0.05, 0.05, 0.02)$$

### Scan Matching Update

Scan matching provides relative pose corrections $(\Delta x, \Delta y, \Delta \theta)$ between consecutive laser scans using Iterative Closest Point (ICP) algorithm. The correction is applied through weighted fusion based on covariance:

$$\mathbf{x}_{updated} = \mathbf{x}_{predicted} + \mathbf{K} \cdot \Delta\mathbf{x}_{scan}$$

Where $\mathbf{K}$ is computed from the relative covariances of the EKF state and scan matching result.

## 2.5 Kalman Filter Equations

### 2.5.1 Prediction Step

**State Prediction:**
$$\bar{\mathbf{x}}_t = g(\mathbf{x}_{t-1}, \mathbf{u}_t)$$

Where $g(\cdot)$ is the motion model function and $\mathbf{u}_t = [v, \omega]^T$ is the control input.

**Covariance Prediction:**
$$\bar{\mathbf{P}}_t = \mathbf{G}_t \mathbf{P}_{t-1} \mathbf{G}_t^T + \mathbf{V}_t \mathbf{M}_t \mathbf{V}_t^T$$

**Jacobian of Motion Model (State):**

For straight-line motion ($|\omega| < 10^{-6}$):
$$\mathbf{G}_t = \begin{bmatrix}
1 & 0 & -v \cdot \sin(\theta_t) \cdot \Delta t \\
0 & 1 & v \cdot \cos(\theta_t) \cdot \Delta t \\
0 & 0 & 1
\end{bmatrix}$$

For arc motion:
$$\mathbf{G}_t = \begin{bmatrix}
1 & 0 & -r \cdot \cos(\theta_t) + r \cdot \cos(\theta_t + \omega \cdot \Delta t) \\
0 & 1 & -r \cdot \sin(\theta_t) + r \cdot \sin(\theta_t + \omega \cdot \Delta t) \\
0 & 0 & 1
\end{bmatrix}$$

Where $r = v/\omega$ is the arc radius.

**Jacobian of Motion Model (Control):**
$$\mathbf{V}_t = \begin{bmatrix}
\cos(\theta_t) \cdot \Delta t & 0 \\
\sin(\theta_t) \cdot \Delta t & 0 \\
0 & \Delta t
\end{bmatrix}$$

**Motion Noise Covariance:**
$$\mathbf{M}_t = \begin{bmatrix}
\alpha_1 v^2 + \alpha_2 \omega^2 & 0 \\
0 & \alpha_3 v^2 + \alpha_4 \omega^2
\end{bmatrix}$$

With noise parameters $\boldsymbol{\alpha} = [\alpha_1, \alpha_2, \alpha_3, \alpha_4] = [0.1, 0.01, 0.01, 0.1]$.

### 2.5.2 Update Step

**Innovation (Measurement Residual):**
$$\mathbf{y}_t = \mathbf{z}_t - \mathbf{H} \bar{\mathbf{x}}_t$$

The angular component of the innovation is normalized to $[-\pi, \pi]$.

**Innovation Covariance:**
$$\mathbf{S}_t = \mathbf{H} \bar{\mathbf{P}}_t \mathbf{H}^T + \mathbf{Q}_t$$

**Kalman Gain:**
$$\mathbf{K}_t = \bar{\mathbf{P}}_t \mathbf{H}^T \mathbf{S}_t^{-1}$$

**State Update:**
$$\mathbf{x}_t = \bar{\mathbf{x}}_t + \mathbf{K}_t \mathbf{y}_t$$

**Covariance Update:**
$$\mathbf{P}_t = (\mathbf{I} - \mathbf{K}_t \mathbf{H}) \bar{\mathbf{P}}_t$$

## 2.6 Track Initialization and Management

### Map Initialization

The occupancy grid map is initialized with the following parameters:
- **Resolution**: 0.05 meters per cell
- **Initial size**: 30m × 30m centered at origin
- **Origin**: $(-15, -15)$ meters (bottom-left corner)

Each cell stores a log-odds value representing occupancy probability:
- $l = 0$: Unknown ($P = 0.5$)
- $l > 0$: Likely occupied ($P > 0.5$)
- $l < 0$: Likely free ($P < 0.5$)

### Dynamic Map Expansion

The map automatically expands when the robot approaches boundaries:
- **Margin threshold**: 2.0 meters from any edge
- Existing map data is preserved during expansion
- Grid indices are recalculated to maintain consistency

### Pose Tracking Initialization

The EKF state is initialized from the first odometry reading:
$$\mathbf{x}_0 = [x_{odom}, y_{odom}, \theta_{odom}]^T$$
$$\mathbf{P}_0 = \mathbf{I}_{3 \times 3} \times 0.01$$

## 2.7 Class-Aware Data Association

### Scan-to-Scan Matching

The ICP-based scan matcher performs data association through nearest-neighbor correspondence:

1. **Point Cloud Conversion**: Laser scans are converted to 2D point clouds in the sensor frame
2. **Correspondence Search**: For each point in the current scan, find the nearest neighbor in the previous scan within a maximum distance threshold (0.3m)
3. **Outlier Rejection**: Point pairs exceeding the correspondence distance are rejected

### Occupancy Grid Update

Ray-casting using Bresenham's line algorithm determines which cells each laser beam traverses:
- Cells along the ray (except endpoint) are marked as free
- The endpoint cell is marked as occupied (if range < max_range - 0.1)

Log-odds update values:
- **Occupied observation**: $l_{occ} = 0.85$
- **Free observation**: $l_{free} = -0.4$
- **Clamping bounds**: $l \in [-5.0, 5.0]$

## 2.8 Handling Occlusion and Missed Detections

### Invalid Measurement Filtering

Laser scan readings are filtered before processing:
- **NaN/Inf values**: Rejected
- **Range violations**: Readings outside $[range_{min}, range_{max}]$ are ignored
- Typical bounds: $range_{min} = 0.1m$, $range_{max} = 12.0m$

### Scan Matching Robustness

The scan matching algorithm includes robustness measures:
- **Minimum point threshold**: At least 10 valid points required for matching
- **Downsampling**: Point clouds are downsampled to 200 points maximum for efficiency
- **Convergence criteria**: 
  - Maximum 30 iterations
  - Tolerance threshold: $10^{-4}$
- **Fitness validation**: Match is accepted only if:
  - Mean correspondence distance < 0.2m
  - At least 30% of points have valid correspondences

### Transform Publishing

The map-to-odometry transform is computed to maintain consistent localization:
$$T_{map \rightarrow odom} = T_{map \rightarrow base} \cdot T_{base \rightarrow odom}$$

This transform corrects for accumulated odometry drift detected through scan matching.

## 2.9 Evaluation Approach

### Quantitative Metrics

The implementation provides several mechanisms for evaluating SLAM performance:

1. **Pose Covariance**: The EKF continuously estimates pose uncertainty through the covariance matrix, providing a measure of localization confidence

2. **Scan Matching Fitness Score**: Each scan match produces a fitness score (mean correspondence distance), indicating alignment quality

3. **Map Quality Indicators**:
   - Cell probability distribution (free/occupied/unknown ratios)
   - Map coverage area vs. robot trajectory length

### Visualization and Debugging

Real-time evaluation through ROS2 topics:
- `/slam/pose`: Robot pose with covariance (PoseWithCovarianceStamped)
- `/map`: Occupancy grid visualization (OccupancyGrid)
- `/tf`: Transform tree (map → odom → base_link)

### Performance Considerations

The system is designed for real-time operation with:
- **Map publish rate**: 1 Hz
- **Pose publish rate**: 10 Hz
- **TF publish rate**: Synchronized with odometry updates

Computational efficiency is achieved through:
- Log-odds representation (additive updates vs. multiplicative)
- Point cloud downsampling for scan matching
- Lazy map expansion (only when needed)
