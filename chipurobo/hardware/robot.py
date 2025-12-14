#!/usr/bin/env python3
"""
ChipuRobot - Main Robot Hardware Interface
Unified robot class that orchestrates all hardware components
"""

import time
import math
import threading
from typing import Dict, Any, Optional

from .gpio_manager import GPIOPinManager
from .motors import L298NMotorDriver
from .encoders import MotorEncoder
from ..sensors.imu import MPU9255_IMU
from ..vision.camera import VisionPositioning

# Core imports with error handling
try:
    import RPi.GPIO as GPIO
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
        
        # Robot state
        self.position = {'x': 0.0, 'y': 0.0, 'heading': 0.0}
        self.position_confidence = {
            'encoders': 1.0,
            'imu': 1.0, 
            'vision': 0.0
        }
        
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
        Execute a mission from the web interface
        
        Args:
            mission_data: Mission configuration from server
        """
        print(f"🎯 Executing mission: {mission_data.get('missionId', 'Unnamed')}")
        
        waypoints = mission_data.get('path', {}).get('waypoints', [])
        robot_config = mission_data.get('robotConfig', {})
        
        # Update robot config
        for key, value in robot_config.items():
            self.config[key] = value
        
        # Start position tracking if not already running
        self.start_position_tracking()
        
        try:
            for i, waypoint in enumerate(waypoints):
                print(f"📍 Moving to waypoint {i+1}: ({waypoint['x']:.2f}, {waypoint['y']:.2f})")
                self.move_to_waypoint(waypoint)
                
                # Small pause between waypoints
                time.sleep(0.5)
                
        except Exception as e:
            print(f"❌ Mission execution error: {e}")
        finally:
            self.stop()
            print("✅ Mission complete")
    
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
    
    def cleanup(self) -> None:
        """Clean up all hardware resources"""
        print("🧹 Cleaning up ChipuRobot...")
        
        self.stop_position_tracking()
        self.stop()
        
        self.motor_driver.cleanup()
        self.left_encoder.cleanup()
        self.right_encoder.cleanup()
        self.vision.cleanup()
        
        if RPI_AVAILABLE:
            GPIO.cleanup()
        
        print("✅ ChipuRobot cleanup complete")