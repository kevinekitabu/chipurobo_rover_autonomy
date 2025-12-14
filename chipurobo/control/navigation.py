#!/usr/bin/env python3
"""
ChipuRobo Navigation Controller
Advanced navigation algorithms for autonomous path following
"""

import math
import numpy as np
from typing import Dict, List, Tuple, Optional, Any
import time
import json


class NavigationController:
    """Advanced navigation controller with obstacle avoidance and path planning"""
    
    def __init__(self, robot_config: Dict[str, Any]):
        """
        Initialize navigation controller
        
        Args:
            robot_config: Robot configuration parameters
        """
        self.config = robot_config
        self.current_position = {'x': 0.0, 'y': 0.0, 'heading': 0.0}
        self.target_waypoint = None
        self.path_waypoints = []
        self.current_waypoint_index = 0
        
        # Navigation parameters
        self.waypoint_tolerance = robot_config.get('waypointTolerance', 1.0)  # feet
        self.max_speed = robot_config.get('maxSpeed', 2.0)  # ft/s
        self.max_turn_rate = robot_config.get('maxTurnRate', 90.0)  # degrees/s
        self.lookahead_distance = robot_config.get('lookaheadDistance', 3.0)  # feet
        
        # Control gains
        self.kp_linear = robot_config.get('kpLinear', 1.0)
        self.kp_angular = robot_config.get('kpAngular', 2.0)
        self.ki_linear = robot_config.get('kiLinear', 0.1)
        self.ki_angular = robot_config.get('kiAngular', 0.1)
        self.kd_linear = robot_config.get('kdLinear', 0.05)
        self.kd_angular = robot_config.get('kdAngular', 0.05)
        
        # PID state
        self.linear_error_integral = 0.0
        self.angular_error_integral = 0.0
        self.prev_linear_error = 0.0
        self.prev_angular_error = 0.0
        self.last_update_time = time.time()
        
        # Obstacle avoidance
        self.obstacle_detection_range = robot_config.get('obstacleRange', 2.0)  # feet
        self.navigation_grid = None
        self.load_navigation_grid()
        
        print("🧭 NavigationController initialized")
    
    def load_navigation_grid(self) -> None:
        """Load navigation grid from file"""
        try:
            import os
            grid_path = os.path.join(
                os.path.dirname(__file__), 
                '../../deploy/pathplanner/navgrid.json'
            )
            
            if os.path.exists(grid_path):
                with open(grid_path, 'r') as f:
                    grid_data = json.load(f)
                    self.navigation_grid = grid_data
                    print(f"✅ Navigation grid loaded: {grid_data['field_size']}")
            else:
                print("⚠️ Navigation grid not found, using default")
                self.navigation_grid = self._create_default_grid()
        except Exception as e:
            print(f"⚠️ Failed to load navigation grid: {e}")
            self.navigation_grid = self._create_default_grid()
    
    def _create_default_grid(self) -> Dict[str, Any]:
        """Create a default navigation grid"""
        return {
            "field_size": {"x": 17.548, "y": 8.052},
            "nodeSizeMeters": 0.3,
            "grid": [[True] * 59 for _ in range(27)]  # All traversable
        }
    
    def set_path(self, waypoints: List[Dict[str, float]]) -> None:
        """
        Set new path waypoints for navigation
        
        Args:
            waypoints: List of waypoint dictionaries with 'x', 'y' coordinates
        """
        self.path_waypoints = waypoints.copy()
        self.current_waypoint_index = 0
        
        if waypoints:
            self.target_waypoint = waypoints[0]
            print(f"🎯 Path set with {len(waypoints)} waypoints")
        else:
            self.target_waypoint = None
            print("⚠️ Empty path provided")
    
    def update_position(self, position: Dict[str, float]) -> None:
        """
        Update current robot position
        
        Args:
            position: Current robot position {'x', 'y', 'heading'}
        """
        self.current_position = position.copy()
    
    def is_path_complete(self) -> bool:
        """Check if robot has reached the end of the path"""
        return (self.current_waypoint_index >= len(self.path_waypoints) and 
                self.target_waypoint is None)
    
    def calculate_control_output(self) -> Tuple[float, float]:
        """
        Calculate control output for path following
        
        Returns:
            Tuple of (forward_speed, turn_rate) normalized to [-1.0, 1.0]
        """
        if not self.target_waypoint:
            return 0.0, 0.0
        
        current_time = time.time()
        dt = current_time - self.last_update_time
        self.last_update_time = current_time
        
        # Check if we've reached the current waypoint
        if self._is_waypoint_reached(self.target_waypoint):
            self._advance_to_next_waypoint()
            
            if not self.target_waypoint:
                return 0.0, 0.0  # Path complete
        
        # Use lookahead controller for smooth path following
        lookahead_point = self._calculate_lookahead_point()
        
        # Calculate errors
        linear_error, angular_error = self._calculate_errors(lookahead_point)
        
        # PID control
        forward_speed, turn_rate = self._calculate_pid_output(
            linear_error, angular_error, dt
        )
        
        # Apply speed limits and obstacle avoidance
        forward_speed, turn_rate = self._apply_constraints(forward_speed, turn_rate)
        
        return forward_speed, turn_rate
    
    def _is_waypoint_reached(self, waypoint: Dict[str, float]) -> bool:
        """Check if robot has reached a waypoint"""
        dx = waypoint['x'] - self.current_position['x']
        dy = waypoint['y'] - self.current_position['y']
        distance = math.sqrt(dx*dx + dy*dy)
        
        return distance < self.waypoint_tolerance
    
    def _advance_to_next_waypoint(self) -> None:
        """Move to the next waypoint in the path"""
        self.current_waypoint_index += 1
        
        if self.current_waypoint_index < len(self.path_waypoints):
            self.target_waypoint = self.path_waypoints[self.current_waypoint_index]
            print(f"📍 Advanced to waypoint {self.current_waypoint_index + 1}/{len(self.path_waypoints)}")
        else:
            self.target_waypoint = None
            print("🏁 Path complete!")
    
    def _calculate_lookahead_point(self) -> Dict[str, float]:
        """
        Calculate lookahead point for smooth path following
        
        Returns:
            Lookahead point coordinates
        """
        if not self.target_waypoint:
            return self.current_position
        
        # Simple lookahead: project ahead on line to target
        dx = self.target_waypoint['x'] - self.current_position['x']
        dy = self.target_waypoint['y'] - self.current_position['y']
        distance_to_target = math.sqrt(dx*dx + dy*dy)
        
        if distance_to_target < self.lookahead_distance:
            # Close to target, use target directly
            return self.target_waypoint
        
        # Project lookahead distance along path
        unit_x = dx / distance_to_target
        unit_y = dy / distance_to_target
        
        lookahead_point = {
            'x': self.current_position['x'] + unit_x * self.lookahead_distance,
            'y': self.current_position['y'] + unit_y * self.lookahead_distance
        }
        
        return lookahead_point
    
    def _calculate_errors(self, target_point: Dict[str, float]) -> Tuple[float, float]:
        """
        Calculate linear and angular errors to target
        
        Args:
            target_point: Target point to navigate to
            
        Returns:
            Tuple of (linear_error, angular_error)
        """
        # Linear error (distance to target)
        dx = target_point['x'] - self.current_position['x']
        dy = target_point['y'] - self.current_position['y']
        linear_error = math.sqrt(dx*dx + dy*dy)
        
        # Angular error (heading to target)
        target_heading = math.degrees(math.atan2(dy, dx))
        angular_error = target_heading - self.current_position['heading']
        
        # Normalize angular error to [-180, 180]
        while angular_error > 180:
            angular_error -= 360
        while angular_error < -180:
            angular_error += 360
        
        return linear_error, angular_error
    
    def _calculate_pid_output(self, linear_error: float, angular_error: float, 
                            dt: float) -> Tuple[float, float]:
        """
        Calculate PID control output
        
        Args:
            linear_error: Distance error to target
            angular_error: Heading error to target (degrees)
            dt: Time delta since last update
            
        Returns:
            Tuple of (forward_speed, turn_rate)
        """
        if dt <= 0:
            dt = 0.05  # Default to 20Hz update rate
        
        # Linear control
        self.linear_error_integral += linear_error * dt
        linear_derivative = (linear_error - self.prev_linear_error) / dt
        
        forward_speed = (
            self.kp_linear * linear_error +
            self.ki_linear * self.linear_error_integral +
            self.kd_linear * linear_derivative
        )
        
        # Angular control
        self.angular_error_integral += angular_error * dt
        angular_derivative = (angular_error - self.prev_angular_error) / dt
        
        turn_rate = (
            self.kp_angular * angular_error +
            self.ki_angular * self.angular_error_integral +
            self.kd_angular * angular_derivative
        )
        
        # Update previous errors
        self.prev_linear_error = linear_error
        self.prev_angular_error = angular_error
        
        # Normalize to motor control range [-1.0, 1.0]
        forward_speed = max(-1.0, min(1.0, forward_speed / self.max_speed))
        turn_rate = max(-1.0, min(1.0, turn_rate / self.max_turn_rate))
        
        return forward_speed, turn_rate
    
    def _apply_constraints(self, forward_speed: float, turn_rate: float) -> Tuple[float, float]:
        """
        Apply speed limits and obstacle avoidance constraints
        
        Args:
            forward_speed: Desired forward speed
            turn_rate: Desired turn rate
            
        Returns:
            Constrained (forward_speed, turn_rate)
        """
        # Reduce speed when turning sharply
        turn_factor = 1.0 - abs(turn_rate) * 0.5
        forward_speed *= turn_factor
        
        # Obstacle avoidance (simplified - check navigation grid)
        if self._detect_obstacle_ahead():
            forward_speed = min(forward_speed, 0.2)  # Slow down near obstacles
            
            # Add avoidance turn if needed
            obstacle_turn = self._calculate_avoidance_turn()
            turn_rate += obstacle_turn
            turn_rate = max(-1.0, min(1.0, turn_rate))
        
        return forward_speed, turn_rate
    
    def _detect_obstacle_ahead(self) -> bool:
        """
        Detect obstacles ahead of robot using navigation grid
        
        Returns:
            True if obstacle detected
        """
        if not self.navigation_grid:
            return False
        
        # Project forward from current position
        heading_rad = math.radians(self.current_position['heading'])
        check_distance = self.obstacle_detection_range
        
        for dist in np.arange(0.5, check_distance, 0.5):
            check_x = self.current_position['x'] + dist * math.cos(heading_rad)
            check_y = self.current_position['y'] + dist * math.sin(heading_rad)
            
            if not self._is_position_traversable(check_x, check_y):
                return True
        
        return False
    
    def _is_position_traversable(self, x: float, y: float) -> bool:
        """
        Check if a position is traversable using navigation grid
        
        Args:
            x, y: Position coordinates in feet
            
        Returns:
            True if position is traversable
        """
        if not self.navigation_grid:
            return True
        
        try:
            grid = self.navigation_grid['grid']
            field_size = self.navigation_grid['field_size']
            node_size = self.navigation_grid.get('nodeSizeMeters', 0.3) * 3.28084  # Convert to feet
            
            # Convert position to grid indices
            grid_x = int(x / node_size)
            grid_y = int(y / node_size)
            
            # Check bounds
            if (grid_x < 0 or grid_x >= len(grid[0]) or 
                grid_y < 0 or grid_y >= len(grid)):
                return False
            
            return grid[grid_y][grid_x]  # True = traversable
            
        except Exception as e:
            print(f"⚠️ Grid check error: {e}")
            return True
    
    def _calculate_avoidance_turn(self) -> float:
        """
        Calculate turn rate for obstacle avoidance
        
        Returns:
            Turn rate for avoidance [-1.0, 1.0]
        """
        # Simple avoidance: check left and right
        heading_rad = math.radians(self.current_position['heading'])
        
        # Check 45 degrees left and right
        left_clear = True
        right_clear = True
        
        for angle_offset in [45, -45]:
            check_heading = heading_rad + math.radians(angle_offset)
            check_x = self.current_position['x'] + 2.0 * math.cos(check_heading)
            check_y = self.current_position['y'] + 2.0 * math.sin(check_heading)
            
            if angle_offset > 0:  # Left
                left_clear = self._is_position_traversable(check_x, check_y)
            else:  # Right
                right_clear = self._is_position_traversable(check_x, check_y)
        
        # Choose direction
        if left_clear and not right_clear:
            return 0.5  # Turn left
        elif right_clear and not left_clear:
            return -0.5  # Turn right
        elif left_clear and right_clear:
            return 0.3  # Prefer slight left turn
        else:
            return 0.0  # Both blocked, stop turning
    
    def get_navigation_status(self) -> Dict[str, Any]:
        """
        Get current navigation status
        
        Returns:
            Navigation status dictionary
        """
        return {
            'current_waypoint_index': self.current_waypoint_index,
            'total_waypoints': len(self.path_waypoints),
            'target_waypoint': self.target_waypoint,
            'path_complete': self.is_path_complete(),
            'current_position': self.current_position,
            'lookahead_distance': self.lookahead_distance,
            'waypoint_tolerance': self.waypoint_tolerance
        }
    
    def reset_navigation(self) -> None:
        """Reset navigation state"""
        self.path_waypoints.clear()
        self.current_waypoint_index = 0
        self.target_waypoint = None
        
        # Reset PID state
        self.linear_error_integral = 0.0
        self.angular_error_integral = 0.0
        self.prev_linear_error = 0.0
        self.prev_angular_error = 0.0
        self.last_update_time = time.time()
        
        print("🔄 Navigation reset")