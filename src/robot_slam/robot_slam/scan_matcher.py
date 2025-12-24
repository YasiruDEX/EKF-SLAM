"""
Scan Matching Implementation using Iterative Closest Point (ICP).
Used for correcting odometry drift by matching consecutive laser scans.
"""

import numpy as np
from typing import Tuple, Optional


class ScanMatcher:
    """
    Simple ICP-based scan matching for 2D laser scans.
    """
    
    def __init__(self, 
                 max_iterations: int = 50,
                 tolerance: float = 1e-4,
                 max_correspondence_distance: float = 0.5):
        """
        Initialize scan matcher.
        
        Args:
            max_iterations: Maximum ICP iterations
            tolerance: Convergence tolerance
            max_correspondence_distance: Max distance for point correspondence
        """
        self.max_iterations = max_iterations
        self.tolerance = tolerance
        self.max_correspondence_distance = max_correspondence_distance
        
        # Previous scan data
        self.prev_points = None
    
    def scan_to_points(self, ranges: np.ndarray, 
                       angle_min: float, 
                       angle_increment: float,
                       range_min: float = 0.1,
                       range_max: float = 12.0) -> np.ndarray:
        """
        Convert laser scan to 2D point cloud in sensor frame.
        
        Args:
            ranges: Array of range measurements
            angle_min: Minimum scan angle
            angle_increment: Angle between beams
            range_min, range_max: Valid range bounds
            
        Returns:
            Nx2 array of [x, y] points
        """
        points = []
        for i, r in enumerate(ranges):
            if np.isnan(r) or np.isinf(r) or r < range_min or r > range_max:
                continue
            
            angle = angle_min + i * angle_increment
            x = r * np.cos(angle)
            y = r * np.sin(angle)
            points.append([x, y])
        
        return np.array(points) if points else np.empty((0, 2))
    
    def transform_points(self, points: np.ndarray, 
                         dx: float, dy: float, dtheta: float) -> np.ndarray:
        """Apply 2D rigid transformation to points."""
        if len(points) == 0:
            return points
        
        cos_t = np.cos(dtheta)
        sin_t = np.sin(dtheta)
        
        R = np.array([[cos_t, -sin_t],
                      [sin_t, cos_t]])
        t = np.array([dx, dy])
        
        return (R @ points.T).T + t
    
    def find_correspondences(self, source: np.ndarray, 
                              target: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Find nearest neighbor correspondences.
        
        Returns:
            source_matched, target_matched, distances
        """
        if len(source) == 0 or len(target) == 0:
            return np.empty((0, 2)), np.empty((0, 2)), np.empty(0)
        
        source_matched = []
        target_matched = []
        distances = []
        
        for p in source:
            # Find nearest neighbor in target
            dists = np.linalg.norm(target - p, axis=1)
            min_idx = np.argmin(dists)
            min_dist = dists[min_idx]
            
            if min_dist < self.max_correspondence_distance:
                source_matched.append(p)
                target_matched.append(target[min_idx])
                distances.append(min_dist)
        
        return (np.array(source_matched), 
                np.array(target_matched), 
                np.array(distances))
    
    def estimate_transform(self, source: np.ndarray, 
                           target: np.ndarray) -> Tuple[float, float, float]:
        """
        Estimate rigid transform from source to target using SVD.
        
        Returns:
            dx, dy, dtheta
        """
        if len(source) < 3 or len(target) < 3:
            return 0.0, 0.0, 0.0
        
        # Compute centroids
        centroid_source = np.mean(source, axis=0)
        centroid_target = np.mean(target, axis=0)
        
        # Center the points
        source_centered = source - centroid_source
        target_centered = target - centroid_target
        
        # Compute cross-covariance matrix
        H = source_centered.T @ target_centered
        
        # SVD
        U, S, Vt = np.linalg.svd(H)
        
        # Rotation matrix
        R = Vt.T @ U.T
        
        # Handle reflection case
        if np.linalg.det(R) < 0:
            Vt[-1, :] *= -1
            R = Vt.T @ U.T
        
        # Extract rotation angle
        dtheta = np.arctan2(R[1, 0], R[0, 0])
        
        # Translation
        t = centroid_target - R @ centroid_source
        
        return t[0], t[1], dtheta
    
    def match(self, current_ranges: np.ndarray,
              angle_min: float,
              angle_increment: float,
              initial_guess: Tuple[float, float, float] = (0, 0, 0),
              range_min: float = 0.1,
              range_max: float = 12.0) -> Tuple[float, float, float, float, bool]:
        """
        Match current scan against previous scan.
        
        Args:
            current_ranges: Current laser scan ranges
            angle_min: Minimum scan angle
            angle_increment: Angle between beams
            initial_guess: (dx, dy, dtheta) initial transform guess
            range_min, range_max: Valid range bounds
            
        Returns:
            (dx, dy, dtheta, fitness_score, success)
        """
        current_points = self.scan_to_points(current_ranges, angle_min, 
                                              angle_increment, range_min, range_max)
        
        # If no previous scan, store current and return no transform
        if self.prev_points is None or len(self.prev_points) < 10:
            self.prev_points = current_points
            return 0.0, 0.0, 0.0, 0.0, False
        
        # Downsample for efficiency
        step = max(1, len(current_points) // 200)
        source = current_points[::step]
        target = self.prev_points[::step]
        
        if len(source) < 10 or len(target) < 10:
            self.prev_points = current_points
            return 0.0, 0.0, 0.0, 0.0, False
        
        # ICP iterations
        dx, dy, dtheta = initial_guess
        
        for iteration in range(self.max_iterations):
            # Transform source points
            transformed = self.transform_points(source, dx, dy, dtheta)
            
            # Find correspondences
            src_matched, tgt_matched, dists = self.find_correspondences(
                transformed, target)
            
            if len(src_matched) < 10:
                break
            
            # Estimate transform from original source to matched target
            source_original = self.transform_points(src_matched, -dx, -dy, -dtheta)
            ddx, ddy, ddtheta = self.estimate_transform(source_original, tgt_matched)
            
            # Update transform
            dx_new = dx + ddx
            dy_new = dy + ddy  
            dtheta_new = dtheta + ddtheta
            
            # Check convergence
            delta = np.sqrt((dx_new - dx)**2 + (dy_new - dy)**2 + (dtheta_new - dtheta)**2)
            dx, dy, dtheta = dx_new, dy_new, dtheta_new
            
            if delta < self.tolerance:
                break
        
        # Calculate fitness score
        transformed = self.transform_points(source, dx, dy, dtheta)
        _, _, dists = self.find_correspondences(transformed, target)
        fitness = np.mean(dists) if len(dists) > 0 else float('inf')
        
        success = fitness < 0.2 and len(dists) > len(source) * 0.3
        
        # Update previous scan
        self.prev_points = current_points
        
        # Return negative transform (current to previous)
        return dx, dy, dtheta, fitness, success
    
    def reset(self):
        """Reset the scan matcher state."""
        self.prev_points = None
