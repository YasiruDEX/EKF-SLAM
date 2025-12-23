"""
Occupancy Grid Mapping Implementation
Uses log-odds probability representation for efficient updates.
"""

import numpy as np
from typing import Tuple, Optional


class OccupancyGrid:
    """
    2D Occupancy Grid Map using log-odds representation.
    
    Each cell stores log-odds of occupancy:
    - l = 0: Unknown (P = 0.5)
    - l > 0: Likely occupied (P > 0.5)
    - l < 0: Likely free (P < 0.5)
    """
    
    def __init__(self,
                 width: float = 20.0,
                 height: float = 20.0,
                 resolution: float = 0.05,
                 origin_x: float = -10.0,
                 origin_y: float = -10.0):
        """
        Initialize occupancy grid.
        
        Args:
            width: Map width in meters
            height: Map height in meters
            resolution: Cell size in meters
            origin_x: X coordinate of map origin (bottom-left)
            origin_y: Y coordinate of map origin (bottom-left)
        """
        self.resolution = resolution
        self.origin_x = origin_x
        self.origin_y = origin_y
        self.width = width
        self.height = height
        
        # Grid dimensions in cells
        self.grid_width = int(width / resolution)
        self.grid_height = int(height / resolution)
        
        # Log-odds grid (initialized to 0 = unknown)
        self.log_odds = np.zeros((self.grid_height, self.grid_width), dtype=np.float32)
        
        # Log-odds update values
        self.l_occ = 0.85   # Log-odds for occupied cell
        self.l_free = -0.4  # Log-odds for free cell
        
        # Clamp values to prevent numerical issues
        self.l_min = -5.0
        self.l_max = 5.0
        
        # Prior probability
        self.l_prior = 0.0
    
    def world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        """Convert world coordinates to grid indices."""
        gx = int((x - self.origin_x) / self.resolution)
        gy = int((y - self.origin_y) / self.resolution)
        return gx, gy
    
    def grid_to_world(self, gx: int, gy: int) -> Tuple[float, float]:
        """Convert grid indices to world coordinates (cell center)."""
        x = self.origin_x + (gx + 0.5) * self.resolution
        y = self.origin_y + (gy + 0.5) * self.resolution
        return x, y
    
    def is_valid_cell(self, gx: int, gy: int) -> bool:
        """Check if grid indices are within bounds."""
        return 0 <= gx < self.grid_width and 0 <= gy < self.grid_height
    
    def update_cell(self, gx: int, gy: int, occupied: bool):
        """Update a single cell with occupancy observation."""
        if not self.is_valid_cell(gx, gy):
            return
        
        if occupied:
            self.log_odds[gy, gx] += self.l_occ
        else:
            self.log_odds[gy, gx] += self.l_free
        
        # Clamp values
        self.log_odds[gy, gx] = np.clip(self.log_odds[gy, gx], self.l_min, self.l_max)
    
    def bresenham_line(self, x0: int, y0: int, x1: int, y1: int) -> list:
        """
        Bresenham's line algorithm to get cells between two points.
        Returns list of (x, y) tuples.
        """
        cells = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        
        while True:
            cells.append((x0, y0))
            
            if x0 == x1 and y0 == y1:
                break
            
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x0 += sx
            if e2 < dx:
                err += dx
                y0 += sy
        
        return cells
    
    def update_with_scan(self, robot_x: float, robot_y: float, robot_theta: float,
                          ranges: np.ndarray, angle_min: float, angle_increment: float,
                          range_min: float = 0.1, range_max: float = 12.0):
        """
        Update occupancy grid using laser scan data.
        
        Args:
            robot_x, robot_y, robot_theta: Robot pose in world frame
            ranges: Array of range measurements
            angle_min: Minimum scan angle (radians)
            angle_increment: Angle between consecutive beams (radians)
            range_min: Minimum valid range
            range_max: Maximum valid range
        """
        robot_gx, robot_gy = self.world_to_grid(robot_x, robot_y)
        
        for i, r in enumerate(ranges):
            # Skip invalid readings
            if np.isnan(r) or np.isinf(r) or r < range_min or r > range_max:
                continue
            
            # Calculate beam angle in world frame
            beam_angle = robot_theta + angle_min + i * angle_increment
            
            # Calculate endpoint in world coordinates
            end_x = robot_x + r * np.cos(beam_angle)
            end_y = robot_y + r * np.sin(beam_angle)
            
            end_gx, end_gy = self.world_to_grid(end_x, end_y)
            
            # Get all cells along the beam using Bresenham's algorithm
            cells = self.bresenham_line(robot_gx, robot_gy, end_gx, end_gy)
            
            # Mark all cells except the last as free
            for j, (gx, gy) in enumerate(cells[:-1]):
                self.update_cell(gx, gy, occupied=False)
            
            # Mark the last cell (hit point) as occupied
            if cells:
                last_gx, last_gy = cells[-1]
                # Only mark as occupied if within max range (not max range reading)
                if r < range_max - 0.1:
                    self.update_cell(last_gx, last_gy, occupied=True)
    
    def get_probability_grid(self) -> np.ndarray:
        """Convert log-odds to probability grid [0, 1]."""
        return 1.0 / (1.0 + np.exp(-self.log_odds))
    
    def get_occupancy_grid_msg_data(self) -> np.ndarray:
        """
        Get occupancy grid data in ROS format.
        Returns int8 array: -1=unknown, 0=free, 100=occupied
        """
        prob = self.get_probability_grid()
        result = np.full((self.grid_height, self.grid_width), -1, dtype=np.int8)
        
        # Free cells (probability < 0.4)
        result[prob < 0.4] = 0
        
        # Occupied cells (probability > 0.6)
        result[prob > 0.6] = 100
        
        # Unknown remains -1
        
        return result.flatten()
    
    def resize_if_needed(self, x: float, y: float, margin: float = 2.0) -> bool:
        """
        Check if point is outside grid and resize if needed.
        Returns True if grid was resized.
        """
        need_resize = False
        new_origin_x = self.origin_x
        new_origin_y = self.origin_y
        new_width = self.width
        new_height = self.height
        
        # Check bounds and expand if needed
        if x < self.origin_x + margin:
            expand = self.origin_x + margin - x + margin
            new_origin_x = self.origin_x - expand
            new_width = self.width + expand
            need_resize = True
        
        if x > self.origin_x + self.width - margin:
            expand = x - (self.origin_x + self.width - margin) + margin
            new_width = self.width + expand
            need_resize = True
        
        if y < self.origin_y + margin:
            expand = self.origin_y + margin - y + margin
            new_origin_y = self.origin_y - expand
            new_height = self.height + expand
            need_resize = True
        
        if y > self.origin_y + self.height - margin:
            expand = y - (self.origin_y + self.height - margin) + margin
            new_height = self.height + expand
            need_resize = True
        
        if need_resize:
            self._resize(new_origin_x, new_origin_y, new_width, new_height)
        
        return need_resize
    
    def _resize(self, new_origin_x: float, new_origin_y: float,
                new_width: float, new_height: float):
        """Resize the grid while preserving existing data."""
        new_grid_width = int(new_width / self.resolution)
        new_grid_height = int(new_height / self.resolution)
        
        new_log_odds = np.zeros((new_grid_height, new_grid_width), dtype=np.float32)
        
        # Calculate offset in cells
        offset_x = int((self.origin_x - new_origin_x) / self.resolution)
        offset_y = int((self.origin_y - new_origin_y) / self.resolution)
        
        # Copy old data to new grid
        for gy in range(self.grid_height):
            for gx in range(self.grid_width):
                new_gx = gx + offset_x
                new_gy = gy + offset_y
                if 0 <= new_gx < new_grid_width and 0 <= new_gy < new_grid_height:
                    new_log_odds[new_gy, new_gx] = self.log_odds[gy, gx]
        
        # Update attributes
        self.origin_x = new_origin_x
        self.origin_y = new_origin_y
        self.width = new_width
        self.height = new_height
        self.grid_width = new_grid_width
        self.grid_height = new_grid_height
        self.log_odds = new_log_odds
