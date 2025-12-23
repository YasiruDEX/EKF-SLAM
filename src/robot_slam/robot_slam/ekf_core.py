"""
Extended Kalman Filter Core Implementation for SLAM
Implements EKF prediction and update steps for robot localization and mapping.
"""

import numpy as np
from typing import Tuple, Optional


class EKFCore:
    """
    Extended Kalman Filter for SLAM.
    
    State vector: [x, y, theta] (robot pose only for simplicity)
    This is an EKF-based localization that maintains robot pose estimate
    while building an occupancy grid map separately.
    """
    
    def __init__(self, 
                 initial_pose: np.ndarray = None,
                 motion_noise: np.ndarray = None,
                 measurement_noise: np.ndarray = None):
        """
        Initialize EKF with optional initial state and noise parameters.
        
        Args:
            initial_pose: [x, y, theta] initial robot pose
            motion_noise: [sigma_v, sigma_omega] velocity noise std devs
            measurement_noise: [sigma_r, sigma_phi] range/bearing noise std devs
        """
        # State: [x, y, theta]
        if initial_pose is None:
            self.state = np.zeros(3)
        else:
            self.state = np.array(initial_pose, dtype=np.float64)
        
        # State covariance matrix (3x3)
        self.covariance = np.eye(3) * 0.01  # Small initial uncertainty
        
        # Motion noise parameters (for velocity model)
        if motion_noise is None:
            self.alpha = np.array([0.1, 0.01, 0.01, 0.1])  # Motion noise params
        else:
            self.alpha = motion_noise
        
        # Measurement noise
        if measurement_noise is None:
            self.Q = np.diag([0.1, 0.1])  # Range and bearing noise
        else:
            self.Q = np.diag(measurement_noise)
    
    def normalize_angle(self, angle: float) -> float:
        """Normalize angle to [-pi, pi]."""
        while angle > np.pi:
            angle -= 2 * np.pi
        while angle < -np.pi:
            angle += 2 * np.pi
        return angle
    
    def predict(self, v: float, omega: float, dt: float) -> Tuple[np.ndarray, np.ndarray]:
        """
        EKF Prediction step using velocity motion model.
        
        Args:
            v: Linear velocity (m/s)
            omega: Angular velocity (rad/s)
            dt: Time step (seconds)
            
        Returns:
            Predicted state and covariance
        """
        x, y, theta = self.state
        
        # Handle straight line motion (omega ≈ 0)
        if abs(omega) < 1e-6:
            # Straight line motion
            x_new = x + v * np.cos(theta) * dt
            y_new = y + v * np.sin(theta) * dt
            theta_new = theta
            
            # Jacobian of motion model w.r.t. state
            G = np.array([
                [1, 0, -v * np.sin(theta) * dt],
                [0, 1,  v * np.cos(theta) * dt],
                [0, 0, 1]
            ])
        else:
            # Arc motion
            r = v / omega
            x_new = x - r * np.sin(theta) + r * np.sin(theta + omega * dt)
            y_new = y + r * np.cos(theta) - r * np.cos(theta + omega * dt)
            theta_new = theta + omega * dt
            
            # Jacobian of motion model w.r.t. state
            G = np.array([
                [1, 0, -r * np.cos(theta) + r * np.cos(theta + omega * dt)],
                [0, 1, -r * np.sin(theta) + r * np.sin(theta + omega * dt)],
                [0, 0, 1]
            ])
        
        theta_new = self.normalize_angle(theta_new)
        
        # Motion noise covariance (simplified)
        # Based on velocity-based motion model noise
        M = np.array([
            [self.alpha[0] * v**2 + self.alpha[1] * omega**2, 0],
            [0, self.alpha[2] * v**2 + self.alpha[3] * omega**2]
        ])
        
        # Jacobian of motion w.r.t. control inputs
        V = np.array([
            [np.cos(theta) * dt, 0],
            [np.sin(theta) * dt, 0],
            [0, dt]
        ])
        
        # Process noise in state space
        R = V @ M @ V.T
        
        # Update state and covariance
        self.state = np.array([x_new, y_new, theta_new])
        self.covariance = G @ self.covariance @ G.T + R
        
        return self.state.copy(), self.covariance.copy()
    
    def update_with_odometry(self, odom_x: float, odom_y: float, odom_theta: float,
                             odom_covariance: Optional[np.ndarray] = None):
        """
        Update state using odometry measurement (pseudo-measurement update).
        
        This treats odometry as a direct measurement of the robot pose,
        useful for correcting drift when odometry is relatively accurate.
        
        Args:
            odom_x, odom_y, odom_theta: Odometry pose measurement
            odom_covariance: 3x3 measurement covariance (optional)
        """
        if odom_covariance is None:
            odom_covariance = np.diag([0.05, 0.05, 0.02])  # Default uncertainty
        
        # Measurement model: z = H * x (identity for direct pose observation)
        H = np.eye(3)
        
        # Innovation (measurement - predicted measurement)
        z = np.array([odom_x, odom_y, odom_theta])
        y = z - self.state
        y[2] = self.normalize_angle(y[2])
        
        # Innovation covariance
        S = H @ self.covariance @ H.T + odom_covariance
        
        # Kalman gain
        K = self.covariance @ H.T @ np.linalg.inv(S)
        
        # Update state and covariance
        self.state = self.state + K @ y
        self.state[2] = self.normalize_angle(self.state[2])
        self.covariance = (np.eye(3) - K @ H) @ self.covariance
        
        return self.state.copy(), self.covariance.copy()
    
    def update_with_scan_matching(self, dx: float, dy: float, dtheta: float,
                                   match_covariance: Optional[np.ndarray] = None):
        """
        Update state using scan matching result (relative pose correction).
        
        Args:
            dx, dy, dtheta: Relative pose correction from scan matching
            match_covariance: 3x3 covariance of the scan match result
        """
        if match_covariance is None:
            match_covariance = np.diag([0.02, 0.02, 0.01])
        
        # Apply correction directly with weighted fusion
        # This is a simplified update treating scan match as a correction
        correction = np.array([dx, dy, dtheta])
        
        # Weighted average based on covariances
        K = self.covariance @ np.linalg.inv(self.covariance + match_covariance)
        
        self.state = self.state + K @ correction
        self.state[2] = self.normalize_angle(self.state[2])
        self.covariance = (np.eye(3) - K) @ self.covariance
        
        return self.state.copy(), self.covariance.copy()
    
    def get_state(self) -> Tuple[np.ndarray, np.ndarray]:
        """Return current state estimate and covariance."""
        return self.state.copy(), self.covariance.copy()
    
    def set_state(self, state: np.ndarray, covariance: np.ndarray = None):
        """Set the current state and optionally covariance."""
        self.state = np.array(state, dtype=np.float64)
        if covariance is not None:
            self.covariance = np.array(covariance, dtype=np.float64)
