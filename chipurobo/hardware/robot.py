#!/usr/bin/env python3
"""
ChipuRobot - Main Robot Hardware Interface
Unified robot class that orchestrates all hardware components
"""

import time
import math
import threading
from typing import Dict, Any, Optional, List

from .gpio_manager import GPIOPinManager
from .motors import L298NMotorDriver
from .encoders import MotorEncoder
from ..sensors.imu import MPU9255_IMU
from ..vision.camera import VisionPositioning
from ..control.navigation import NavigationController
from ..control.trajectory import TrajectoryPlanner
from ..control.obstacle_avoidance import ObstacleAvoidanceSystem

# Core imports with error handling
try:
    from gpiozero import Device
    RPI_AVAILABLE = True
except ImportError:
    RPI_AVAILABLE = False


class ChipuRobot:
    """
    Unified ChipuRobot hardware interface
    Combines motor control, encoders, IMU, and vision in a single class
    """
    
    def __init__(self, config: Optional[Dict[str, Any]] = None):
        """
        Initialize ChipuRobot with optional configuration
        
        Args:
            config: Configuration dictionary with robot parameters
        """
        self.config = config or {}
        print("🤖 Initializing ChipuRobot...")
        
        # Print GPIO pin assignments
        GPIOPinManager.print_pin_assignment()
        
        # Initialize motor driver
        self.motor_driver = L298NMotorDriver(
            pwm_freq=self.config.get('pwmFreq', 1000)
        )
        
        # Initialize encoders
        left_pins, right_pins = GPIOPinManager.get_encoder_pins()
        self.left_encoder = MotorEncoder(
            pin_a=left_pins['channel_a'], 
            pin_b=left_pins['channel_b'],
            pulses_per_revolution=self.config.get('encoderPPR', 11),
            wheel_diameter=self.config.get('wheelDiameter', 4.0)
        )
        self.right_encoder = MotorEncoder(
            pin_a=right_pins['channel_a'],
            pin_b=right_pins['channel_b'], 
            pulses_per_revolution=self.config.get('encoderPPR', 11),
            wheel_diameter=self.config.get('wheelDiameter', 4.0)
        )
        
        # Initialize IMU
        self.imu = MPU9255_IMU()
        
        # Initialize vision system
        self.vision = VisionPositioning()
        
        # Initialize advanced control systems
        self.navigation_controller = NavigationController(self.config)
        self.trajectory_planner = TrajectoryPlanner(self.config)
        self.obstacle_avoidance = ObstacleAvoidanceSystem(self.config)
        
        # Robot state
        self.position = {'x': 0.0, 'y': 0.0, 'heading': 0.0}
        self.position_confidence = {
            'encoders': 1.0,
            'imu': 1.0, 
            'vision': 0.0
        }
        
        # Mission state
        self.current_trajectory = None
        self.mission_start_time = None
        self.autonomous_mode = False
        
        # Thread for position updates
        self.position_thread = None
        self.running = False
        
        print("✅ ChipuRobot initialization complete!")
        self.print_capabilities()
    
    def print_capabilities(self) -> None:
        """Print robot capabilities based on available hardware"""
        print("🔧 Robot Capabilities:")
        print(f"   ⚙️ Motor Control: {'✅ Active' if self.motor_driver.initialized or not RPI_AVAILABLE else '❌ Failed'}")
        print(f"   📏 Left Encoder: {'✅ Active' if self.left_encoder.active or not RPI_AVAILABLE else '❌ Failed'}")
        print(f"   📏 Right Encoder: {'✅ Active' if self.right_encoder.active or not RPI_AVAILABLE else '❌ Failed'}")
        print(f"   🧭 IMU: {'✅ Active' if self.imu.available or not RPI_AVAILABLE else '❌ Failed'}")
        print(f"   📷 Vision: {'✅ Active' if self.vision.available or not RPI_AVAILABLE else '❌ Failed'}")
        print("🎯 Advanced Control Systems:")
        print(f"   🧭 Navigation Controller: ✅ Active")
        print(f"   📊 Trajectory Planner: ✅ Active")
        print(f"   🛡️ Obstacle Avoidance: ✅ Active")
    
    def start_position_tracking(self) -> None:
        """Start background thread for position tracking"""
        if self.position_thread and self.position_thread.is_alive():
            return
            
        self.running = True
        self.position_thread = threading.Thread(target=self._position_update_loop)
        self.position_thread.daemon = True
        self.position_thread.start()
        print("📍 Position tracking started")
    
    def stop_position_tracking(self) -> None:
        """Stop position tracking thread"""
        self.running = False
        if self.position_thread:
            self.position_thread.join(timeout=1.0)
        print("📍 Position tracking stopped")
    
    def _position_update_loop(self) -> None:
        """Background position update using sensor fusion"""
        last_time = time.time()
        
        while self.running:
            try:
                current_time = time.time()
                dt = current_time - last_time
                
                if dt >= 0.05:  # Update at 20Hz
                    self.update_position_fusion(dt)
                    last_time = current_time
                
                time.sleep(0.01)  # Small sleep to prevent excessive CPU usage
                
            except Exception as e:
                print(f"❌ Position update error: {e}")
                time.sleep(0.1)
    
    def update_position_fusion(self, dt: float) -> None:
        """
        Update robot position using sensor fusion
        
        Args:
            dt: Time delta since last update
        """
        # 1. Get encoder-based movement
        left_dist = self.left_encoder.get_distance() / 12.0  # Convert to feet
        right_dist = self.right_encoder.get_distance() / 12.0
        
        # Calculate movement from encoders (differential drive)
        distance = (left_dist + right_dist) / 2.0
        delta_heading = (right_dist - left_dist) / (self.config.get('wheelBase', 12.0) / 12.0)
        
        # 2. Get IMU heading correction
        imu_data = self.imu.get_all_data()
        imu_heading_rate = imu_data['gyro']['z']  # Z-axis rotation in rad/s
        imu_delta_heading = imu_heading_rate * dt
        
        # 3. Sensor fusion for heading (weighted average)
        encoder_weight = self.position_confidence['encoders']
        imu_weight = self.position_confidence['imu']
        
        total_weight = encoder_weight + imu_weight
        if total_weight > 0:
            fused_delta_heading = (encoder_weight * delta_heading + imu_weight * imu_delta_heading) / total_weight
        else:
            fused_delta_heading = delta_heading
        
        # 4. Update position
        self.position['heading'] += math.degrees(fused_delta_heading)
        self.position['heading'] = self.position['heading'] % 360  # Keep in 0-360 range
        
        # Update X,Y position
        heading_rad = math.radians(self.position['heading'])
        self.position['x'] += distance * math.cos(heading_rad)
        self.position['y'] += distance * math.sin(heading_rad)
        
        # 5. Vision correction (if available)
        vision_pos = self.vision.get_position_from_markers()
        if vision_pos and self.position_confidence['vision'] > 0:
            vision_weight = self.position_confidence['vision']
            total_pos_weight = 1.0 + vision_weight
            
            # Blend vision with encoder position
            self.position['x'] = (self.position['x'] + vision_weight * vision_pos[0]) / total_pos_weight
            self.position['y'] = (self.position['y'] + vision_weight * vision_pos[1]) / total_pos_weight
            self.position['heading'] = (self.position['heading'] + vision_weight * vision_pos[2]) / total_pos_weight
        
        # Reset encoder counts for next update
        self.left_encoder.reset()
        self.right_encoder.reset()
    
    def get_position(self) -> Dict[str, float]:
        """Get current robot position"""
        return self.position.copy()
    
    def get_sensor_data(self) -> Dict[str, Any]:
        """Get all sensor readings"""
        return {
            'position': self.get_position(),
            'encoders': {
                'left': self.left_encoder.get_status(),
                'right': self.right_encoder.get_status()
            },
            'imu': self.imu.get_all_data(),
            'vision': self.vision.get_status(),
            'motor_driver': self.motor_driver.get_status()
        }
    
    def get_system_status(self) -> Dict[str, Any]:
        """Get comprehensive system status"""
        return {
            'robot_id': 'ChipuRobot-v1.0',
            'config': self.config,
            'hardware': {
                'gpio_manager': GPIOPinManager.get_pin_usage_summary(),
                'motor_driver': self.motor_driver.get_status(),
                'encoders': {
                    'left': self.left_encoder.get_status(),
                    'right': self.right_encoder.get_status()
                },
                'imu': self.imu.get_status(),
                'vision': self.vision.get_status()
            },
            'position_tracking': {
                'running': self.running,
                'position': self.get_position(),
                'confidence': self.position_confidence
            }
        }
    
    # Motor control methods (delegate to motor driver)
    def drive_tank(self, left_speed: float, right_speed: float) -> None:
        """Tank drive control (-1.0 to 1.0 for each side)"""
        self.motor_driver.drive_tank(left_speed, right_speed)
    
    def drive_arcade(self, forward_speed: float, turn_rate: float) -> None:
        """Arcade drive control (-1.0 to 1.0 for forward and turn)"""
        self.motor_driver.drive_arcade(forward_speed, turn_rate)
    
    def stop(self) -> None:
        """Stop all motors"""
        self.motor_driver.stop()
    
    def execute_mission(self, mission_data: Dict[str, Any]) -> None:
        """
        Execute a mission using advanced trajectory planning and navigation
        
        Args:
            mission_data: Mission configuration from server
        """
        print(f"🎯 Executing advanced mission: {mission_data.get('missionId', 'Unnamed')}")
        
        waypoints = mission_data.get('path', {}).get('waypoints', [])
        robot_config = mission_data.get('robotConfig', {})
        use_advanced_navigation = mission_data.get('useAdvancedNavigation', True)
        
        # Update robot config
        for key, value in robot_config.items():
            self.config[key] = value
        
        # Start position tracking if not already running
        self.start_position_tracking()
        
        try:
            if use_advanced_navigation and len(waypoints) > 1:
                self.execute_advanced_mission(waypoints)
            else:
                self.execute_basic_mission(waypoints)
                
        except Exception as e:
            print(f"❌ Mission execution error: {e}")
        finally:
            self.stop()
            self.autonomous_mode = False
            print("✅ Mission complete")
    
    def execute_advanced_mission(self, waypoints: List[Dict[str, float]]) -> None:
        """
        Execute mission using advanced trajectory planning and navigation
        
        Args:
            waypoints: List of waypoints to navigate through
        """
        print("🚀 Starting advanced autonomous navigation")
        
        # Generate smooth trajectory
        print("📊 Generating trajectory...")
        trajectory = self.trajectory_planner.generate_trajectory(waypoints)
        self.current_trajectory = trajectory
        
        # Set navigation controller path
        self.navigation_controller.set_path(waypoints)
        
        # Enable autonomous mode
        self.autonomous_mode = True
        self.mission_start_time = time.time()
        
        # Main navigation loop
        control_frequency = 20  # Hz
        loop_time = 1.0 / control_frequency
        
        while not self.navigation_controller.is_path_complete() and self.autonomous_mode:
            loop_start = time.time()
            
            # Update systems with current position
            current_position = self.get_position()
            self.navigation_controller.update_position(current_position)
            
            # Update obstacle avoidance with sensor data
            sensor_data = self.get_sensor_data()
            self.obstacle_avoidance.update_sensor_data(sensor_data)
            
            # Calculate navigation control output
            forward_speed, turn_rate = self.navigation_controller.calculate_control_output()
            
            # Apply obstacle avoidance if obstacles detected
            if self.obstacle_avoidance.current_obstacles:
                target_waypoint = self.navigation_controller.target_waypoint
                if target_waypoint:
                    forward_speed, turn_rate = self.obstacle_avoidance.calculate_avoidance_control(
                        current_position, target_waypoint, forward_speed, turn_rate
                    )
            
            # Apply control output
            self.drive_arcade(forward_speed, turn_rate)
            
            # Status update
            if int(time.time()) % 2 == 0:  # Every 2 seconds
                nav_status = self.navigation_controller.get_navigation_status()
                obstacle_status = self.obstacle_avoidance.get_obstacle_status()
                print(f"📍 Waypoint {nav_status['current_waypoint_index'] + 1}/"
                      f"{nav_status['total_waypoints']}, "
                      f"Obstacles: {obstacle_status['num_obstacles']}")
            
            # Maintain loop frequency
            elapsed = time.time() - loop_start
            if elapsed < loop_time:
                time.sleep(loop_time - elapsed)
        
        print("🏁 Advanced navigation complete")
    
    def execute_basic_mission(self, waypoints: List[Dict[str, float]]) -> None:
        """
        Execute mission using basic waypoint navigation (fallback)
        
        Args:
            waypoints: List of waypoints to navigate through
        """
        print("🎯 Starting basic waypoint navigation")
        
        for i, waypoint in enumerate(waypoints):
            print(f"📍 Moving to waypoint {i+1}: ({waypoint['x']:.2f}, {waypoint['y']:.2f})")
            self.move_to_waypoint(waypoint)
            
            # Small pause between waypoints
            time.sleep(0.5)
    
    def execute_trajectory_following(self, target_time: float = None) -> None:
        """
        Execute trajectory following with time optimization
        
        Args:
            target_time: Optional target completion time for optimization
        """
        if not self.current_trajectory:
            print("❌ No trajectory available")
            return
        
        print("📊 Starting trajectory following")
        
        trajectory = self.current_trajectory
        if target_time:
            print(f"⏱️ Optimizing for target time: {target_time}s")
            trajectory = self.trajectory_planner.optimize_trajectory_for_time(
                trajectory['waypoints'], target_time
            )
        
        # Execute timed trajectory
        start_time = time.time()
        
        while True:
            elapsed_time = time.time() - start_time
            
            # Get target point at current time
            target_point = self.trajectory_planner.get_trajectory_point_at_time(
                trajectory, elapsed_time
            )
            
            if not target_point:
                break
            
            # Navigate to target point
            current_position = self.get_position()
            
            # Calculate errors and control
            dx = target_point['x'] - current_position['x']
            dy = target_point['y'] - current_position['y']
            distance_error = math.sqrt(dx*dx + dy*dy)
            
            target_heading = math.degrees(math.atan2(dy, dx))
            heading_error = target_heading - current_position['heading']
            
            # Normalize heading error
            while heading_error > 180:
                heading_error -= 360
            while heading_error < -180:
                heading_error += 360
            
            # Simple proportional control
            forward_speed = min(1.0, distance_error * 0.5)
            turn_rate = max(-1.0, min(1.0, heading_error / 90.0))
            
            self.drive_arcade(forward_speed, turn_rate)
            time.sleep(0.05)  # 20Hz control
        
        print("✅ Trajectory following complete")
    
    def move_to_waypoint(self, waypoint: Dict[str, float]) -> None:
        """
        Move to a specific waypoint using position feedback
        
        Args:
            waypoint: Dictionary with 'x' and 'y' coordinates
        """
        target_x = waypoint['x']
        target_y = waypoint['y']
        tolerance = 2.0  # inches
        
        max_speed = self.config.get('maxSpeed', 2.0) / 6.0  # Convert ft/s to motor scale
        
        while True:
            current_pos = self.get_position()
            
            # Calculate distance and angle to target
            dx = target_x - current_pos['x']
            dy = target_y - current_pos['y']
            distance = math.sqrt(dx*dx + dy*dy)
            
            if distance < tolerance:
                print(f"✅ Reached waypoint: ({target_x:.2f}, {target_y:.2f})")
                break
            
            # Calculate target heading
            target_heading = math.degrees(math.atan2(dy, dx))
            
            # Calculate heading error
            heading_error = target_heading - current_pos['heading']
            
            # Normalize heading error to -180 to 180
            while heading_error > 180:
                heading_error -= 360
            while heading_error < -180:
                heading_error += 360
            
            # Simple proportional control
            turn_rate = max(-1.0, min(1.0, heading_error / 90.0))  # Turn rate based on heading error
            forward_speed = max_speed * (1.0 - abs(turn_rate) * 0.5)  # Reduce speed while turning
            
            self.drive_arcade(forward_speed, turn_rate)
            time.sleep(0.1)  # Control loop frequency
        
        self.stop()
    
    def reset_position(self) -> None:
        """Reset robot position to origin"""
        self.position = {'x': 0.0, 'y': 0.0, 'heading': 0.0}
        self.left_encoder.reset()
        self.right_encoder.reset()
        print("📍 Position reset to origin")
    
    def calibrate_sensors(self) -> Dict[str, Any]:
        """Calibrate all sensors"""
        results = {}
        
        print("🔧 Starting sensor calibration...")
        
        # IMU calibration
        if self.imu.available:
            results['imu'] = self.imu.calibrate()
        else:
            results['imu'] = {'status': 'not_available'}
        
        # Encoder calibration (reset counts)
        self.left_encoder.reset()
        self.right_encoder.reset()
        results['encoders'] = {'status': 'reset'}
        
        # Vision system check
        if self.vision.available:
            results['vision'] = {'status': 'available'}
        else:
            results['vision'] = {'status': 'not_available'}
        
        print("✅ Sensor calibration complete")
        return results
    
    def stop_autonomous_mission(self) -> None:
        """Stop current autonomous mission"""
        self.autonomous_mode = False
        self.stop()
        print("⏹️ Autonomous mission stopped")
    
    def get_advanced_navigation_status(self) -> Dict[str, Any]:
        """Get comprehensive navigation system status"""
        nav_status = self.navigation_controller.get_navigation_status()
        obstacle_status = self.obstacle_avoidance.get_obstacle_status()
        
        trajectory_stats = {}
        if self.current_trajectory:
            trajectory_stats = self.trajectory_planner.get_trajectory_statistics(
                self.current_trajectory
            )
        
        return {
            'navigation': nav_status,
            'obstacles': obstacle_status,
            'trajectory': trajectory_stats,
            'autonomous_mode': self.autonomous_mode,
            'mission_elapsed_time': (
                time.time() - self.mission_start_time 
                if self.mission_start_time else 0
            )
        }
    
    def plan_optimal_path(self, waypoints: List[Dict[str, float]], 
                         constraints: Dict[str, Any] = None) -> Dict[str, Any]:
        """
        Plan optimal trajectory for given waypoints
        
        Args:
            waypoints: Target waypoints
            constraints: Optional trajectory constraints
            
        Returns:
            Planned trajectory with statistics
        """
        print(f"🗺️ Planning optimal path through {len(waypoints)} waypoints")
        
        trajectory = self.trajectory_planner.generate_trajectory(waypoints, constraints)
        stats = self.trajectory_planner.get_trajectory_statistics(trajectory)
        
        print(f"✅ Path planned: {stats['total_distance']:.2f}ft in {stats['total_time']:.2f}s")
        
        return {
            'trajectory': trajectory,
            'statistics': stats,
            'feasible': stats['max_velocity'] <= self.config.get('maxVelocity', 3.0)
        }
    
    def test_obstacle_avoidance(self) -> None:
        """Test obstacle avoidance system with simulated obstacles"""
        print("🧪 Testing obstacle avoidance system")
        
        # Simulate obstacles
        test_sensor_data = {
            'ultrasonic': {
                'front': 2.0,
                'front_left': float('inf'),
                'front_right': 1.5,
                'left': float('inf'),
                'right': float('inf')
            }
        }
        
        self.obstacle_avoidance.update_sensor_data(test_sensor_data)
        
        current_pos = self.get_position()
        target_pos = {'x': current_pos['x'] + 5.0, 'y': current_pos['y']}
        
        # Test avoidance calculation
        forward_speed, turn_rate = self.obstacle_avoidance.calculate_avoidance_control(
            current_pos, target_pos, 0.5, 0.0
        )
        
        print(f"🛡️ Avoidance test - Modified control: forward={forward_speed:.2f}, turn={turn_rate:.2f}")
        
        # Test path clearance
        path_clear = self.obstacle_avoidance.is_path_clear(current_pos, target_pos)
        print(f"🛤️ Path clear: {path_clear}")
        
        # Reset obstacle data
        self.obstacle_avoidance.reset_obstacle_tracking()
        print("✅ Obstacle avoidance test complete")
    
    def emergency_stop(self) -> None:
        """Emergency stop - immediately halt all systems"""
        print("🚨 EMERGENCY STOP ACTIVATED")
        
        self.autonomous_mode = False
        self.stop()
        
        # Reset navigation systems
        self.navigation_controller.reset_navigation()
        self.obstacle_avoidance.reset_obstacle_tracking()
        
        print("🛑 All systems halted")
    
    def cleanup(self) -> None:
        """Clean up all hardware resources"""
        print("🧹 Cleaning up ChipuRobot...")
        
        # Stop autonomous operations
        self.autonomous_mode = False
        self.stop_position_tracking()
        self.stop()
        
        # Reset advanced control systems
        if hasattr(self, 'navigation_controller'):
            self.navigation_controller.reset_navigation()
        if hasattr(self, 'obstacle_avoidance'):
            self.obstacle_avoidance.reset_obstacle_tracking()
        
        # Cleanup hardware
        self.motor_driver.cleanup()
        self.left_encoder.cleanup()
        self.right_encoder.cleanup()
        self.vision.cleanup()
        
        if RPI_AVAILABLE:
            import RPi.GPIO as GPIO
            GPIO.cleanup()
        
        print("✅ ChipuRobot cleanup complete")