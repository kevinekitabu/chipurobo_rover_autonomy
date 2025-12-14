#!/usr/bin/env python3
"""
ChipuRobo Trajectory Planner
Advanced trajectory planning with path smoothing and velocity profiling
"""

import math
import numpy as np
from typing import Dict, List, Tuple, Optional, Any
import scipy.interpolate as interp
from scipy.optimize import minimize_scalar


class TrajectoryPlanner:
    """Advanced trajectory planner with path smoothing and velocity profiles"""
    
    def __init__(self, robot_config: Dict[str, Any]):
        """
        Initialize trajectory planner
        
        Args:
            robot_config: Robot configuration parameters
        """
        self.config = robot_config
        
        # Robot physical constraints
        self.max_velocity = robot_config.get('maxVelocity', 3.0)  # ft/s
        self.max_acceleration = robot_config.get('maxAcceleration', 2.0)  # ft/s²
        self.max_angular_velocity = robot_config.get('maxAngularVelocity', 180.0)  # deg/s
        self.max_angular_acceleration = robot_config.get('maxAngularAcceleration', 360.0)  # deg/s²
        
        # Path smoothing parameters
        self.smoothing_resolution = robot_config.get('smoothingResolution', 0.1)  # feet
        self.corner_radius = robot_config.get('cornerRadius', 1.0)  # feet
        self.spline_smoothness = robot_config.get('splineSmoothness', 0.1)
        
        # Trajectory parameters
        self.time_resolution = robot_config.get('timeResolution', 0.05)  # seconds
        self.velocity_lookahead = robot_config.get('velocityLookahead', 2.0)  # feet
        
        print("📊 TrajectoryPlanner initialized")
    
    def generate_trajectory(self, waypoints: List[Dict[str, float]], 
                          constraints: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
        """
        Generate complete trajectory from waypoints
        
        Args:
            waypoints: List of waypoint dictionaries with 'x', 'y' coordinates
            constraints: Optional trajectory constraints
            
        Returns:
            Complete trajectory with path, velocities, and timing
        """
        if len(waypoints) < 2:
            raise ValueError("Need at least 2 waypoints for trajectory generation")
        
        print(f"🛤️ Generating trajectory for {len(waypoints)} waypoints")
        
        # Step 1: Generate smooth path
        smooth_path = self._generate_smooth_path(waypoints)
        
        # Step 2: Calculate curvature profile
        curvature_profile = self._calculate_curvature(smooth_path)
        
        # Step 3: Generate velocity profile
        velocity_profile = self._generate_velocity_profile(smooth_path, curvature_profile)
        
        # Step 4: Generate time-based trajectory
        timed_trajectory = self._generate_timed_trajectory(
            smooth_path, velocity_profile
        )
        
        trajectory = {
            'waypoints': waypoints,
            'smooth_path': smooth_path,
            'curvature_profile': curvature_profile,
            'velocity_profile': velocity_profile,
            'timed_trajectory': timed_trajectory,
            'total_time': timed_trajectory[-1]['time'] if timed_trajectory else 0.0,
            'total_distance': self._calculate_path_distance(smooth_path)
        }
        
        print(f"✅ Trajectory generated: {trajectory['total_distance']:.2f}ft in {trajectory['total_time']:.2f}s")
        return trajectory
    
    def _generate_smooth_path(self, waypoints: List[Dict[str, float]]) -> List[Dict[str, float]]:
        """
        Generate smooth path using cubic spline interpolation
        
        Args:
            waypoints: Original waypoints
            
        Returns:
            Smooth path with higher resolution
        """
        if len(waypoints) < 2:
            return waypoints
        
        # Extract coordinates
        x_coords = [wp['x'] for wp in waypoints]
        y_coords = [wp['y'] for wp in waypoints]
        
        # Handle special case of 2 points (straight line)
        if len(waypoints) == 2:
            return self._generate_straight_line(waypoints[0], waypoints[1])
        
        # Calculate cumulative distances for parameterization
        distances = [0.0]
        for i in range(1, len(waypoints)):
            dx = x_coords[i] - x_coords[i-1]
            dy = y_coords[i] - y_coords[i-1]
            distances.append(distances[-1] + math.sqrt(dx*dx + dy*dy))
        
        try:
            # Create cubic splines
            spline_x = interp.CubicSpline(distances, x_coords, bc_type='natural')
            spline_y = interp.CubicSpline(distances, y_coords, bc_type='natural')
            
            # Generate smooth path
            total_distance = distances[-1]
            num_points = max(int(total_distance / self.smoothing_resolution), len(waypoints))
            smooth_distances = np.linspace(0, total_distance, num_points)
            
            smooth_path = []
            for d in smooth_distances:
                smooth_path.append({
                    'x': float(spline_x(d)),
                    'y': float(spline_y(d)),
                    'distance': float(d)
                })
            
            return smooth_path
            
        except Exception as e:
            print(f"⚠️ Spline generation failed: {e}, using linear interpolation")
            return self._generate_linear_interpolation(waypoints)
    
    def _generate_straight_line(self, start: Dict[str, float], 
                              end: Dict[str, float]) -> List[Dict[str, float]]:
        """Generate straight line between two points"""
        dx = end['x'] - start['x']
        dy = end['y'] - start['y']
        distance = math.sqrt(dx*dx + dy*dy)
        
        num_points = max(int(distance / self.smoothing_resolution), 2)
        path = []
        
        for i in range(num_points):
            t = i / (num_points - 1)
            path.append({
                'x': start['x'] + t * dx,
                'y': start['y'] + t * dy,
                'distance': t * distance
            })
        
        return path
    
    def _generate_linear_interpolation(self, waypoints: List[Dict[str, float]]) -> List[Dict[str, float]]:
        """Generate linear interpolation between waypoints"""
        smooth_path = []
        total_distance = 0.0
        
        for i in range(len(waypoints) - 1):
            start = waypoints[i]
            end = waypoints[i + 1]
            
            dx = end['x'] - start['x']
            dy = end['y'] - start['y']
            segment_distance = math.sqrt(dx*dx + dy*dy)
            
            num_points = max(int(segment_distance / self.smoothing_resolution), 2)
            
            for j in range(num_points):
                if i == len(waypoints) - 2 and j == num_points - 1:
                    # Last point
                    smooth_path.append({
                        'x': end['x'],
                        'y': end['y'],
                        'distance': total_distance + segment_distance
                    })
                else:
                    t = j / (num_points - 1)
                    smooth_path.append({
                        'x': start['x'] + t * dx,
                        'y': start['y'] + t * dy,
                        'distance': total_distance + t * segment_distance
                    })
            
            total_distance += segment_distance
        
        return smooth_path
    
    def _calculate_curvature(self, path: List[Dict[str, float]]) -> List[float]:
        """
        Calculate curvature at each point along the path
        
        Args:
            path: Smooth path points
            
        Returns:
            Curvature values (1/radius) at each point
        """
        if len(path) < 3:
            return [0.0] * len(path)
        
        curvatures = []
        
        for i in range(len(path)):
            if i == 0 or i == len(path) - 1:
                curvatures.append(0.0)  # Straight at endpoints
                continue
            
            # Use three points to calculate curvature
            p1 = path[i - 1]
            p2 = path[i]
            p3 = path[i + 1]
            
            # Calculate curvature using the formula for three points
            curvature = self._calculate_three_point_curvature(p1, p2, p3)
            curvatures.append(abs(curvature))
        
        return curvatures
    
    def _calculate_three_point_curvature(self, p1: Dict[str, float], 
                                       p2: Dict[str, float], 
                                       p3: Dict[str, float]) -> float:
        """Calculate curvature using three points"""
        # Vector from p1 to p2
        v1x = p2['x'] - p1['x']
        v1y = p2['y'] - p1['y']
        
        # Vector from p2 to p3
        v2x = p3['x'] - p2['x']
        v2y = p3['y'] - p2['y']
        
        # Calculate the cross product and magnitudes
        cross_product = v1x * v2y - v1y * v2x
        v1_mag = math.sqrt(v1x*v1x + v1y*v1y)
        v2_mag = math.sqrt(v2x*v2x + v2y*v2y)
        
        if v1_mag == 0 or v2_mag == 0:
            return 0.0
        
        # Calculate angle between vectors
        dot_product = v1x * v2x + v1y * v2y
        cos_angle = dot_product / (v1_mag * v2_mag)
        cos_angle = max(-1.0, min(1.0, cos_angle))  # Clamp to valid range
        
        angle_change = math.acos(cos_angle)
        average_length = (v1_mag + v2_mag) / 2.0
        
        if average_length == 0:
            return 0.0
        
        curvature = angle_change / average_length
        return curvature if cross_product >= 0 else -curvature
    
    def _generate_velocity_profile(self, path: List[Dict[str, float]], 
                                 curvatures: List[float]) -> List[float]:
        """
        Generate velocity profile based on curvature and constraints
        
        Args:
            path: Smooth path points
            curvatures: Curvature at each point
            
        Returns:
            Velocity profile (ft/s) for each point
        """
        if not path:
            return []
        
        velocities = []
        
        # Calculate maximum velocity based on curvature
        for i, curvature in enumerate(curvatures):
            # Velocity limited by centripetal acceleration
            if curvature > 0:
                max_curve_velocity = math.sqrt(
                    self.max_acceleration / max(curvature, 0.001)
                )
            else:
                max_curve_velocity = self.max_velocity
            
            # Apply global velocity limit
            velocity = min(max_curve_velocity, self.max_velocity)
            velocities.append(velocity)
        
        # Forward pass: limit acceleration
        for i in range(1, len(velocities)):
            distance = path[i]['distance'] - path[i-1]['distance']
            max_velocity_from_accel = math.sqrt(
                velocities[i-1]**2 + 2 * self.max_acceleration * distance
            )
            velocities[i] = min(velocities[i], max_velocity_from_accel)
        
        # Backward pass: limit deceleration
        for i in range(len(velocities) - 2, -1, -1):
            distance = path[i+1]['distance'] - path[i]['distance']
            max_velocity_from_decel = math.sqrt(
                velocities[i+1]**2 + 2 * self.max_acceleration * distance
            )
            velocities[i] = min(velocities[i], max_velocity_from_decel)
        
        # Ensure we start and end at low velocity
        velocities[0] = min(velocities[0], 0.5)  # Start slowly
        velocities[-1] = min(velocities[-1], 0.5)  # End slowly
        
        return velocities
    
    def _generate_timed_trajectory(self, path: List[Dict[str, float]], 
                                 velocities: List[float]) -> List[Dict[str, Any]]:
        """
        Generate time-based trajectory points
        
        Args:
            path: Smooth path points
            velocities: Velocity profile
            
        Returns:
            Timed trajectory with positions, velocities, and times
        """
        if not path or not velocities:
            return []
        
        timed_trajectory = []
        current_time = 0.0
        
        for i in range(len(path)):
            # Calculate heading
            if i < len(path) - 1:
                dx = path[i+1]['x'] - path[i]['x']
                dy = path[i+1]['y'] - path[i]['y']
                heading = math.degrees(math.atan2(dy, dx))
            else:
                # Use previous heading for last point
                heading = timed_trajectory[-1]['heading'] if timed_trajectory else 0.0
            
            timed_trajectory.append({
                'time': current_time,
                'x': path[i]['x'],
                'y': path[i]['y'],
                'heading': heading,
                'velocity': velocities[i],
                'distance': path[i]['distance']
            })
            
            # Calculate time to next point
            if i < len(path) - 1:
                distance = path[i+1]['distance'] - path[i]['distance']
                average_velocity = (velocities[i] + velocities[i+1]) / 2.0
                
                if average_velocity > 0:
                    dt = distance / average_velocity
                    current_time += dt
        
        return timed_trajectory
    
    def _calculate_path_distance(self, path: List[Dict[str, float]]) -> float:
        """Calculate total distance of path"""
        if not path:
            return 0.0
        return path[-1].get('distance', 0.0)
    
    def get_trajectory_point_at_time(self, trajectory: Dict[str, Any], 
                                   target_time: float) -> Optional[Dict[str, Any]]:
        """
        Get trajectory point at specific time using interpolation
        
        Args:
            trajectory: Generated trajectory
            target_time: Time to query
            
        Returns:
            Interpolated trajectory point or None if out of bounds
        """
        timed_traj = trajectory['timed_trajectory']
        
        if not timed_traj or target_time < 0:
            return None
        
        if target_time >= timed_traj[-1]['time']:
            return timed_traj[-1]
        
        # Find bounding points
        for i in range(len(timed_traj) - 1):
            if timed_traj[i]['time'] <= target_time <= timed_traj[i+1]['time']:
                # Linear interpolation
                t1, t2 = timed_traj[i]['time'], timed_traj[i+1]['time']
                
                if t2 == t1:
                    return timed_traj[i]
                
                alpha = (target_time - t1) / (t2 - t1)
                
                return {
                    'time': target_time,
                    'x': timed_traj[i]['x'] + alpha * (timed_traj[i+1]['x'] - timed_traj[i]['x']),
                    'y': timed_traj[i]['y'] + alpha * (timed_traj[i+1]['y'] - timed_traj[i]['y']),
                    'heading': self._interpolate_angle(timed_traj[i]['heading'], 
                                                     timed_traj[i+1]['heading'], alpha),
                    'velocity': timed_traj[i]['velocity'] + alpha * (timed_traj[i+1]['velocity'] - timed_traj[i]['velocity']),
                    'distance': timed_traj[i]['distance'] + alpha * (timed_traj[i+1]['distance'] - timed_traj[i]['distance'])
                }
        
        return timed_traj[0]
    
    def _interpolate_angle(self, angle1: float, angle2: float, alpha: float) -> float:
        """Interpolate between two angles correctly handling wraparound"""
        diff = angle2 - angle1
        
        # Handle wraparound
        if diff > 180:
            diff -= 360
        elif diff < -180:
            diff += 360
        
        result = angle1 + alpha * diff
        
        # Normalize result
        while result > 180:
            result -= 360
        while result < -180:
            result += 360
        
        return result
    
    def optimize_trajectory_for_time(self, waypoints: List[Dict[str, float]], 
                                   target_time: float) -> Dict[str, Any]:
        """
        Optimize trajectory to complete in target time
        
        Args:
            waypoints: Path waypoints
            target_time: Desired completion time
            
        Returns:
            Optimized trajectory
        """
        def time_objective(scale_factor):
            """Objective function for time optimization"""
            temp_config = self.config.copy()
            temp_config['maxVelocity'] = self.max_velocity * scale_factor
            temp_config['maxAcceleration'] = self.max_acceleration * scale_factor
            
            temp_planner = TrajectoryPlanner(temp_config)
            temp_trajectory = temp_planner.generate_trajectory(waypoints)
            
            return abs(temp_trajectory['total_time'] - target_time)
        
        # Optimize velocity scaling
        result = minimize_scalar(time_objective, bounds=(0.1, 5.0), method='bounded')
        
        if result.success:
            optimal_scale = result.x
            optimized_config = self.config.copy()
            optimized_config['maxVelocity'] = self.max_velocity * optimal_scale
            optimized_config['maxAcceleration'] = self.max_acceleration * optimal_scale
            
            optimized_planner = TrajectoryPlanner(optimized_config)
            return optimized_planner.generate_trajectory(waypoints)
        else:
            print("⚠️ Trajectory optimization failed, using default")
            return self.generate_trajectory(waypoints)
    
    def get_trajectory_statistics(self, trajectory: Dict[str, Any]) -> Dict[str, Any]:
        """
        Calculate trajectory statistics
        
        Args:
            trajectory: Generated trajectory
            
        Returns:
            Statistics dictionary
        """
        timed_traj = trajectory['timed_trajectory']
        
        if not timed_traj:
            return {}
        
        velocities = [point['velocity'] for point in timed_traj]
        curvatures = trajectory['curvature_profile']
        
        return {
            'total_time': trajectory['total_time'],
            'total_distance': trajectory['total_distance'],
            'average_velocity': sum(velocities) / len(velocities),
            'max_velocity': max(velocities),
            'max_curvature': max(curvatures) if curvatures else 0.0,
            'num_trajectory_points': len(timed_traj),
            'smoothness_factor': trajectory['total_distance'] / len(trajectory['waypoints'])
        }