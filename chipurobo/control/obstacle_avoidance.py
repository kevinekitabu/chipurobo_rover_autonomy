#!/usr/bin/env python3
"""
ChipuRobo Advanced Obstacle Avoidance System
Real-time obstacle detection and dynamic path replanning
"""

import math
import numpy as np
from typing import Dict, List, Tuple, Optional, Any
import time
import threading


class ObstacleAvoidanceSystem:
    """Advanced obstacle avoidance with dynamic path replanning"""
    
    def __init__(self, robot_config: Dict[str, Any]):
        """
        Initialize obstacle avoidance system
        
        Args:
            robot_config: Robot configuration parameters
        """
        self.config = robot_config
        
        # Detection parameters
        self.detection_range = robot_config.get('obstacleDetectionRange', 3.0)  # feet
        self.detection_angle = robot_config.get('obstacleDetectionAngle', 60.0)  # degrees
        self.safety_margin = robot_config.get('safetyMargin', 1.0)  # feet
        
        # Dynamic obstacle tracking
        self.dynamic_obstacles = []
        self.obstacle_history = []
        self.max_history_size = 10
        
        # Avoidance parameters
        self.avoidance_strength = robot_config.get('avoidanceStrength', 1.0)
        self.goal_attraction_strength = robot_config.get('goalAttractionStrength', 2.0)
        self.max_avoidance_turn = robot_config.get('maxAvoidanceTurn', 0.8)
        
        # State
        self.last_obstacle_check = 0.0
        self.check_frequency = 0.1  # seconds
        self.current_obstacles = []
        
        print("🛡️ ObstacleAvoidanceSystem initialized")
    
    def update_sensor_data(self, sensor_data: Dict[str, Any]) -> None:
        """
        Update with latest sensor data for obstacle detection
        
        Args:
            sensor_data: Latest sensor readings (ultrasonic, lidar, camera, etc.)
        """
        current_time = time.time()
        
        if current_time - self.last_obstacle_check < self.check_frequency:
            return
        
        self.last_obstacle_check = current_time
        
        # Extract obstacle information from sensors
        new_obstacles = []
        
        # Process ultrasonic sensor data
        if 'ultrasonic' in sensor_data:
            ultrasonic_obstacles = self._process_ultrasonic_data(sensor_data['ultrasonic'])
            new_obstacles.extend(ultrasonic_obstacles)
        
        # Process camera/vision data
        if 'vision' in sensor_data:
            vision_obstacles = self._process_vision_data(sensor_data['vision'])
            new_obstacles.extend(vision_obstacles)
        
        # Process lidar data (if available)
        if 'lidar' in sensor_data:
            lidar_obstacles = self._process_lidar_data(sensor_data['lidar'])
            new_obstacles.extend(lidar_obstacles)
        
        # Update obstacle tracking
        self._update_obstacle_tracking(new_obstacles, current_time)
    
    def _process_ultrasonic_data(self, ultrasonic_data: Dict[str, float]) -> List[Dict[str, Any]]:
        """Process ultrasonic sensor data for obstacle detection"""
        obstacles = []
        
        # Assume ultrasonic sensors mounted at different angles
        sensor_positions = {
            'front': {'angle': 0, 'range': ultrasonic_data.get('front', float('inf'))},
            'front_left': {'angle': 30, 'range': ultrasonic_data.get('front_left', float('inf'))},
            'front_right': {'angle': -30, 'range': ultrasonic_data.get('front_right', float('inf'))},
            'left': {'angle': 90, 'range': ultrasonic_data.get('left', float('inf'))},
            'right': {'angle': -90, 'range': ultrasonic_data.get('right', float('inf'))}
        }
        
        for sensor_name, sensor_info in sensor_positions.items():
            distance = sensor_info['range']
            
            # Check if obstacle detected within range
            if distance < self.detection_range and distance > 0.1:  # Ignore very close readings
                angle_rad = math.radians(sensor_info['angle'])
                
                # Convert to relative coordinates
                obstacle_x = distance * math.cos(angle_rad)
                obstacle_y = distance * math.sin(angle_rad)
                
                obstacles.append({
                    'type': 'ultrasonic',
                    'x': obstacle_x,
                    'y': obstacle_y,
                    'distance': distance,
                    'angle': sensor_info['angle'],
                    'confidence': 0.8,
                    'sensor': sensor_name
                })
        
        return obstacles
    
    def _process_vision_data(self, vision_data: Dict[str, Any]) -> List[Dict[str, Any]]:
        """Process camera/vision data for obstacle detection"""
        obstacles = []
        
        # Process detected objects from computer vision
        detected_objects = vision_data.get('objects', [])
        
        for obj in detected_objects:
            # Skip if it's a known landmark or goal
            if obj.get('type') in ['aruco_marker', 'goal', 'waypoint']:
                continue
            
            # Extract position and size
            distance = obj.get('distance', 0.0)
            angle = obj.get('angle', 0.0)
            size = obj.get('size', 0.5)
            
            if distance > 0 and distance < self.detection_range:
                angle_rad = math.radians(angle)
                obstacle_x = distance * math.cos(angle_rad)
                obstacle_y = distance * math.sin(angle_rad)
                
                obstacles.append({
                    'type': 'vision',
                    'x': obstacle_x,
                    'y': obstacle_y,
                    'distance': distance,
                    'angle': angle,
                    'size': size,
                    'confidence': obj.get('confidence', 0.6),
                    'object_type': obj.get('type', 'unknown')
                })
        
        return obstacles
    
    def _process_lidar_data(self, lidar_data: Dict[str, Any]) -> List[Dict[str, Any]]:
        """Process LIDAR data for obstacle detection"""
        obstacles = []
        
        # Process LIDAR scan points
        scan_points = lidar_data.get('scan_points', [])
        
        for point in scan_points:
            distance = point.get('distance', 0.0)
            angle = point.get('angle', 0.0)
            
            if distance > 0.1 and distance < self.detection_range:
                angle_rad = math.radians(angle)
                obstacle_x = distance * math.cos(angle_rad)
                obstacle_y = distance * math.sin(angle_rad)
                
                obstacles.append({
                    'type': 'lidar',
                    'x': obstacle_x,
                    'y': obstacle_y,
                    'distance': distance,
                    'angle': angle,
                    'confidence': 0.9,
                    'intensity': point.get('intensity', 1.0)
                })
        
        return obstacles
    
    def _update_obstacle_tracking(self, new_obstacles: List[Dict[str, Any]], 
                                current_time: float) -> None:
        """Update obstacle tracking with new detections"""
        # Cluster nearby obstacles
        clustered_obstacles = self._cluster_obstacles(new_obstacles)
        
        # Update dynamic obstacle tracking
        self._track_dynamic_obstacles(clustered_obstacles, current_time)
        
        # Update current obstacle list
        self.current_obstacles = clustered_obstacles
        
        # Add to history
        self.obstacle_history.append({
            'time': current_time,
            'obstacles': clustered_obstacles.copy()
        })
        
        # Trim history
        if len(self.obstacle_history) > self.max_history_size:
            self.obstacle_history.pop(0)
    
    def _cluster_obstacles(self, obstacles: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
        """Cluster nearby obstacle detections"""
        if not obstacles:
            return []
        
        clustered = []
        cluster_distance = 0.5  # feet
        
        for obstacle in obstacles:
            # Find if this obstacle is close to an existing cluster
            merged = False
            
            for cluster in clustered:
                dx = obstacle['x'] - cluster['x']
                dy = obstacle['y'] - cluster['y']
                distance = math.sqrt(dx*dx + dy*dy)
                
                if distance < cluster_distance:
                    # Merge with existing cluster (weighted average)
                    total_confidence = cluster['confidence'] + obstacle['confidence']
                    
                    cluster['x'] = (cluster['x'] * cluster['confidence'] + 
                                  obstacle['x'] * obstacle['confidence']) / total_confidence
                    cluster['y'] = (cluster['y'] * cluster['confidence'] + 
                                  obstacle['y'] * obstacle['confidence']) / total_confidence
                    cluster['confidence'] = min(1.0, total_confidence)
                    cluster['size'] = max(cluster.get('size', 0.3), obstacle.get('size', 0.3))
                    
                    merged = True
                    break
            
            if not merged:
                clustered.append(obstacle.copy())
        
        return clustered
    
    def _track_dynamic_obstacles(self, current_obstacles: List[Dict[str, Any]], 
                               current_time: float) -> None:
        """Track dynamic obstacles and estimate velocities"""
        if len(self.obstacle_history) < 2:
            return
        
        prev_obstacles = self.obstacle_history[-1]['obstacles']
        dt = current_time - self.obstacle_history[-1]['time']
        
        if dt <= 0:
            return
        
        # Match current obstacles with previous ones
        for curr_obs in current_obstacles:
            best_match = None
            min_distance = float('inf')
            
            for prev_obs in prev_obstacles:
                dx = curr_obs['x'] - prev_obs['x']
                dy = curr_obs['y'] - prev_obs['y']
                distance = math.sqrt(dx*dx + dy*dy)
                
                if distance < min_distance and distance < 1.0:  # Max movement in one update
                    min_distance = distance
                    best_match = prev_obs
            
            if best_match:
                # Calculate velocity
                dx = curr_obs['x'] - best_match['x']
                dy = curr_obs['y'] - best_match['y']
                
                curr_obs['velocity_x'] = dx / dt
                curr_obs['velocity_y'] = dy / dt
                curr_obs['speed'] = math.sqrt(dx*dx + dy*dy) / dt
                curr_obs['is_dynamic'] = curr_obs['speed'] > 0.5  # ft/s threshold
    
    def calculate_avoidance_force(self, robot_position: Dict[str, float], 
                                target_position: Dict[str, float]) -> Tuple[float, float]:
        """
        Calculate avoidance force using artificial potential field
        
        Args:
            robot_position: Current robot position {'x', 'y', 'heading'}
            target_position: Target/goal position {'x', 'y'}
            
        Returns:
            Tuple of (force_x, force_y) in robot coordinate frame
        """
        total_force_x = 0.0
        total_force_y = 0.0
        
        # Convert robot heading to radians
        robot_heading_rad = math.radians(robot_position['heading'])
        
        # Calculate repulsive forces from obstacles
        for obstacle in self.current_obstacles:
            # Transform obstacle to global coordinates
            obstacle_global_x = (robot_position['x'] + 
                               obstacle['x'] * math.cos(robot_heading_rad) - 
                               obstacle['y'] * math.sin(robot_heading_rad))
            obstacle_global_y = (robot_position['y'] + 
                               obstacle['x'] * math.sin(robot_heading_rad) + 
                               obstacle['y'] * math.cos(robot_heading_rad))
            
            # Calculate repulsive force
            dx = robot_position['x'] - obstacle_global_x
            dy = robot_position['y'] - obstacle_global_y
            distance = math.sqrt(dx*dx + dy*dy)
            
            if distance > 0 and distance < self.detection_range:
                # Repulsive force (inverse square law with safety margin)
                force_magnitude = (self.avoidance_strength * obstacle['confidence'] / 
                                 (distance + self.safety_margin)**2)
                
                # Account for obstacle size
                obstacle_size = obstacle.get('size', 0.3)
                force_magnitude *= (1.0 + obstacle_size)
                
                # Predict future obstacle position if dynamic
                if obstacle.get('is_dynamic', False):
                    prediction_time = distance / 2.0  # Simple prediction
                    future_x = obstacle_global_x + obstacle.get('velocity_x', 0) * prediction_time
                    future_y = obstacle_global_y + obstacle.get('velocity_y', 0) * prediction_time
                    
                    # Use future position for force calculation
                    dx = robot_position['x'] - future_x
                    dy = robot_position['y'] - future_y
                    distance = max(distance, math.sqrt(dx*dx + dy*dy))
                
                # Normalize direction
                if distance > 0:
                    force_x = force_magnitude * dx / distance
                    force_y = force_magnitude * dy / distance
                    
                    total_force_x += force_x
                    total_force_y += force_y
        
        # Add attractive force toward goal
        goal_dx = target_position['x'] - robot_position['x']
        goal_dy = target_position['y'] - robot_position['y']
        goal_distance = math.sqrt(goal_dx*goal_dx + goal_dy*goal_dy)
        
        if goal_distance > 0:
            goal_force_magnitude = self.goal_attraction_strength
            
            # Normalize goal force
            goal_force_x = goal_force_magnitude * goal_dx / goal_distance
            goal_force_y = goal_force_magnitude * goal_dy / goal_distance
            
            total_force_x += goal_force_x
            total_force_y += goal_force_y
        
        return total_force_x, total_force_y
    
    def calculate_avoidance_control(self, robot_position: Dict[str, float],
                                  target_position: Dict[str, float],
                                  current_forward_speed: float,
                                  current_turn_rate: float) -> Tuple[float, float]:
        """
        Calculate modified control output with obstacle avoidance
        
        Args:
            robot_position: Current robot position
            target_position: Target position
            current_forward_speed: Current forward speed command
            current_turn_rate: Current turn rate command
            
        Returns:
            Modified (forward_speed, turn_rate) with avoidance
        """
        # Get avoidance force
        force_x, force_y = self.calculate_avoidance_force(robot_position, target_position)
        
        # Convert force to robot coordinate frame
        robot_heading_rad = math.radians(robot_position['heading'])
        
        # Rotate force to robot frame
        force_robot_x = (force_x * math.cos(robot_heading_rad) + 
                        force_y * math.sin(robot_heading_rad))
        force_robot_y = (-force_x * math.sin(robot_heading_rad) + 
                        force_y * math.cos(robot_heading_rad))
        
        # Calculate avoidance adjustments
        avoidance_forward = force_robot_x * 0.5  # Scale factor
        avoidance_turn = force_robot_y * 0.5     # Scale factor
        
        # Limit avoidance turn rate
        avoidance_turn = max(-self.max_avoidance_turn, 
                           min(self.max_avoidance_turn, avoidance_turn))
        
        # Apply avoidance to current commands
        modified_forward_speed = current_forward_speed + avoidance_forward
        modified_turn_rate = current_turn_rate + avoidance_turn
        
        # Ensure we don't exceed limits
        modified_forward_speed = max(-1.0, min(1.0, modified_forward_speed))
        modified_turn_rate = max(-1.0, min(1.0, modified_turn_rate))
        
        # Slow down if obstacles are very close
        min_obstacle_distance = self._get_minimum_obstacle_distance()
        if min_obstacle_distance < self.safety_margin:
            speed_reduction = max(0.1, min_obstacle_distance / self.safety_margin)
            modified_forward_speed *= speed_reduction
        
        return modified_forward_speed, modified_turn_rate
    
    def _get_minimum_obstacle_distance(self) -> float:
        """Get distance to closest obstacle"""
        if not self.current_obstacles:
            return float('inf')
        
        min_distance = float('inf')
        for obstacle in self.current_obstacles:
            distance = math.sqrt(obstacle['x']**2 + obstacle['y']**2)
            min_distance = min(min_distance, distance)
        
        return min_distance
    
    def is_path_clear(self, robot_position: Dict[str, float], 
                     target_position: Dict[str, float], 
                     path_width: float = 1.0) -> bool:
        """
        Check if path to target is clear of obstacles
        
        Args:
            robot_position: Current robot position
            target_position: Target position to check path to
            path_width: Width of path corridor to check
            
        Returns:
            True if path is clear
        """
        # Calculate path vector
        path_dx = target_position['x'] - robot_position['x']
        path_dy = target_position['y'] - robot_position['y']
        path_length = math.sqrt(path_dx*path_dx + path_dy*path_dy)
        
        if path_length == 0:
            return True
        
        # Normalize path direction
        path_unit_x = path_dx / path_length
        path_unit_y = path_dy / path_length
        
        # Check obstacles along path
        for obstacle in self.current_obstacles:
            # Transform obstacle to global coordinates
            robot_heading_rad = math.radians(robot_position['heading'])
            obstacle_global_x = (robot_position['x'] + 
                               obstacle['x'] * math.cos(robot_heading_rad) - 
                               obstacle['y'] * math.sin(robot_heading_rad))
            obstacle_global_y = (robot_position['y'] + 
                               obstacle['x'] * math.sin(robot_heading_rad) + 
                               obstacle['y'] * math.cos(robot_heading_rad))
            
            # Calculate distance from obstacle to path line
            obstacle_dx = obstacle_global_x - robot_position['x']
            obstacle_dy = obstacle_global_y - robot_position['y']
            
            # Project obstacle onto path direction
            projection = obstacle_dx * path_unit_x + obstacle_dy * path_unit_y
            
            # Check if obstacle is along the path
            if 0 <= projection <= path_length:
                # Calculate perpendicular distance to path
                perp_distance = abs(obstacle_dx * path_unit_y - obstacle_dy * path_unit_x)
                
                # Check if obstacle intersects path corridor
                obstacle_size = obstacle.get('size', 0.3)
                if perp_distance < (path_width / 2.0 + obstacle_size + self.safety_margin):
                    return False
        
        return True
    
    def get_obstacle_status(self) -> Dict[str, Any]:
        """
        Get current obstacle detection status
        
        Returns:
            Status dictionary with obstacle information
        """
        return {
            'num_obstacles': len(self.current_obstacles),
            'obstacles': self.current_obstacles.copy(),
            'min_distance': self._get_minimum_obstacle_distance(),
            'detection_range': self.detection_range,
            'safety_margin': self.safety_margin,
            'dynamic_obstacles': sum(1 for obs in self.current_obstacles 
                                   if obs.get('is_dynamic', False))
        }
    
    def reset_obstacle_tracking(self) -> None:
        """Reset all obstacle tracking data"""
        self.current_obstacles.clear()
        self.dynamic_obstacles.clear()
        self.obstacle_history.clear()
        print("🔄 Obstacle tracking reset")