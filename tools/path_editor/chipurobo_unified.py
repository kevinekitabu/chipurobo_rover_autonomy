#!/usr/bin/env python3
"""
ChipuRobo - Unified Robot Hardware Interface
Consolidated implementation with L298N motor driver, encoders, IMU, and camera
Fixed GPIO pin conflicts and modular hardware detection
"""

import time
import math
import threading
import json
from typing import Dict, Any, Optional, Tuple

# Core imports with error handling
try:
    import RPi.GPIO as GPIO
    RPI_AVAILABLE = True
    print("✅ RPi.GPIO available")
except ImportError:
    print("⚠️ RPi.GPIO not available - running in simulation mode")
    RPI_AVAILABLE = False

# IMU imports
try:
    import board
    import busio
    import adafruit_mpu6050
    IMU_AVAILABLE = True
    print("✅ IMU libraries available")
except ImportError:
    print("⚠️ IMU libraries not available - install: pip install adafruit-circuitpython-mpu6050")
    IMU_AVAILABLE = False

# Camera and vision imports
try:
    import cv2
    import picamera2
    import numpy as np
    CAMERA_AVAILABLE = True
    print("✅ Camera libraries available")
except ImportError:
    print("⚠️ Camera libraries not available - install: pip install opencv-python picamera2")
    CAMERA_AVAILABLE = False


class GPIOPinManager:
    """Centralized GPIO pin assignment manager to prevent conflicts"""
    
    # Standardized pin assignments (avoiding conflicts)
    PINS = {
        # L298N Motor Driver
        'left_motor': {
            'pwm': 18,    # ENA - PWM control
            'in1': 24,    # IN1 - Direction 1
            'in2': 23     # IN2 - Direction 2
        },
        'right_motor': {
            'pwm': 12,    # ENB - PWM control (changed from 19 to avoid encoder conflict)
            'in1': 22,    # IN3 - Direction 1 (changed from 21 to spread out)
            'in2': 27     # IN4 - Direction 2 (changed from 20 to avoid conflicts)
        },
        # Motor Encoders (built-in hall effect sensors)
        'left_encoder': {
            'channel_a': 5,   # Left encoder A
            'channel_b': 6    # Left encoder B
        },
        'right_encoder': {
            'channel_a': 13,  # Right encoder A
            'channel_b': 19   # Right encoder B (no conflict now)
        },
        # IMU (I2C - uses dedicated pins)
        'imu': {
            'sda': 2,  # I2C Data (board pin 3)
            'scl': 3   # I2C Clock (board pin 5)
        },
        # Camera (CSI interface - dedicated pins, no GPIO conflicts)
        'camera': 'CSI',
        # Optional servo for camera gimbal
        'camera_servo': 25,
        # Status LED
        'status_led': 26
    }
    
    @classmethod
    def get_motor_pins(cls):
        return cls.PINS['left_motor'], cls.PINS['right_motor']
    
    @classmethod
    def get_encoder_pins(cls):
        return cls.PINS['left_encoder'], cls.PINS['right_encoder']
    
    @classmethod
    def print_pin_assignment(cls):
        print("🔌 GPIO Pin Assignment:")
        print(f"   Left Motor:  PWM={cls.PINS['left_motor']['pwm']}, IN1={cls.PINS['left_motor']['in1']}, IN2={cls.PINS['left_motor']['in2']}")
        print(f"   Right Motor: PWM={cls.PINS['right_motor']['pwm']}, IN1={cls.PINS['right_motor']['in1']}, IN2={cls.PINS['right_motor']['in2']}")
        print(f"   Left Encoder: A={cls.PINS['left_encoder']['channel_a']}, B={cls.PINS['left_encoder']['channel_b']}")
        print(f"   Right Encoder: A={cls.PINS['right_encoder']['channel_a']}, B={cls.PINS['right_encoder']['channel_b']}")
        print(f"   IMU: I2C SDA={cls.PINS['imu']['sda']}, SCL={cls.PINS['imu']['scl']}")
        print(f"   Camera: {cls.PINS['camera']} interface")


class MotorEncoder:
    """Built-in motor encoder interface (Hall effect encoders in DC motors)"""
    
    def __init__(self, pin_a: int, pin_b: int, pulses_per_revolution: int = 11, 
                 wheel_diameter: float = 4.0, gear_ratio: float = 1.0):
        """
        Initialize motor encoder
        
        Args:
            pin_a, pin_b: GPIO pins for encoder channels A and B
            pulses_per_revolution: PPR of encoder (typically 11-40 for geared motors)
            wheel_diameter: Wheel diameter in inches
            gear_ratio: Motor gearbox ratio (if applicable)
        """
        self.pin_a = pin_a
        self.pin_b = pin_b
        self.ppr = pulses_per_revolution
        self.wheel_diameter = wheel_diameter
        self.gear_ratio = gear_ratio
        self.wheel_circumference = math.pi * wheel_diameter
        
        # Thread-safe counters
        self.count = 0
        self.count_lock = threading.Lock()
        self.last_time = time.time()
        self.velocity = 0.0  # inches per second
        self.active = False
        
        if RPI_AVAILABLE:
            self.setup_gpio()
        else:
            print(f"🔄 Encoder (simulation): Pin A={pin_a}, Pin B={pin_b}")
    
    def setup_gpio(self):
        """Setup GPIO pins and interrupts"""
        try:
            GPIO.setup(self.pin_a, GPIO.IN, pull_up_down=GPIO.PUD_UP)
            GPIO.setup(self.pin_b, GPIO.IN, pull_up_down=GPIO.PUD_UP)
            
            # Add interrupt for precise counting
            GPIO.add_event_detect(self.pin_a, GPIO.BOTH, callback=self._encoder_callback, bouncetime=1)
            self.active = True
            print(f"🔄 Encoder initialized: Pin A={self.pin_a}, Pin B={self.pin_b}")
        except Exception as e:
            print(f"❌ Encoder setup failed: {e}")
    
    def _encoder_callback(self, channel):
        """Interrupt callback for encoder pulses"""
        if not self.active:
            return
            
        current_time = time.time()
        dt = current_time - self.last_time
        
        with self.count_lock:
            # Read both pins to determine direction
            pin_a_state = GPIO.input(self.pin_a)
            pin_b_state = GPIO.input(self.pin_b)
            
            # Quadrature encoding - determine direction
            if pin_a_state == pin_b_state:
                self.count += 1  # Forward
            else:
                self.count -= 1  # Backward
            
            # Calculate velocity
            if dt > 0.001:  # Avoid division by zero
                pulse_rate = 1.0 / dt  # pulses per second
                self.velocity = (pulse_rate / self.ppr) * self.wheel_circumference
        
        self.last_time = current_time
    
    def get_distance(self) -> float:
        """Get distance traveled in inches (thread-safe)"""
        with self.count_lock:
            revolutions = self.count / self.ppr
            return revolutions * self.wheel_circumference
    
    def get_velocity(self) -> float:
        """Get current velocity in inches/second"""
        return self.velocity
    
    def reset(self):
        """Reset encoder count"""
        with self.count_lock:
            self.count = 0
    
    def cleanup(self):
        """Clean up GPIO resources"""
        self.active = False
        if RPI_AVAILABLE:
            try:
                GPIO.remove_event_detect(self.pin_a)
            except Exception:
                pass


class MPU9255_IMU:
    """MPU9255 9-axis IMU interface via I2C"""
    
    def __init__(self):
        self.imu = None
        self.available = False
        
        if IMU_AVAILABLE and RPI_AVAILABLE:
            self.setup_imu()
        else:
            print("🧭 IMU running in simulation mode")
    
    def setup_imu(self):
        """Initialize I2C and IMU"""
        try:
            i2c = busio.I2C(board.SCL, board.SDA)
            self.imu = adafruit_mpu6050.MPU6050(i2c)
            self.available = True
            print("🧭 MPU9255 IMU initialized on I2C")
        except Exception as e:
            print(f"❌ IMU setup failed: {e}")
    
    def get_acceleration(self) -> Tuple[float, float, float]:
        """Get acceleration in m/s² (x, y, z)"""
        if self.available:
            try:
                return self.imu.acceleration
            except Exception:
                pass
        return (0.0, 0.0, 9.81)  # Simulate stationary
    
    def get_gyro(self) -> Tuple[float, float, float]:
        """Get angular velocity in rad/s (x, y, z)"""
        if self.available:
            try:
                return self.imu.gyro
            except Exception:
                pass
        return (0.0, 0.0, 0.0)  # Simulate no rotation
    
    def get_heading_degrees(self) -> float:
        """Get heading in degrees (integrated from gyro Z)"""
        if self.available:
            try:
                gyro_z = self.imu.gyro[2]  # rad/s
                # Simple integration - could be improved with Kalman filter
                return math.degrees(gyro_z)
            except Exception:
                pass
        return 0.0


class VisionPositioning:
    """Computer vision positioning using ArUco markers and Raspberry Pi camera"""
    
    def __init__(self):
        self.camera = None
        self.available = False
        self.aruco_dict = None
        self.aruco_params = None
        
        if CAMERA_AVAILABLE and RPI_AVAILABLE:
            self.setup_camera()
        else:
            print("📷 Camera running in simulation mode")
    
    def setup_camera(self):
        """Initialize Pi camera and ArUco detection"""
        try:
            self.camera = picamera2.Picamera2()
            config = self.camera.create_still_configuration(main={"size": (640, 480)})
            self.camera.configure(config)
            self.camera.start()
            
            # Setup ArUco detection
            self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
            self.aruco_params = cv2.aruco.DetectorParameters()
            
            self.available = True
            print("📷 Pi Camera initialized for vision positioning")
        except Exception as e:
            print(f"❌ Camera setup failed: {e}")
    
    def get_position_from_markers(self) -> Optional[Tuple[float, float, float]]:
        """Get position from ArUco markers (x, y, heading)"""
        if not self.available:
            return None
        
        try:
            # Capture image
            frame = self.camera.capture_array()
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            
            # Detect ArUco markers
            detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
            corners, ids, rejected = detector.detectMarkers(gray)
            
            if ids is not None and len(ids) > 0:
                # Calculate position based on marker detection
                # This is a simplified implementation - real world would need calibration
                marker_center = corners[0][0].mean(axis=0)
                x_pos = (marker_center[0] - 320) / 32.0  # Convert pixels to inches
                y_pos = (marker_center[1] - 240) / 32.0
                
                # Calculate heading from marker orientation
                heading = math.atan2(corners[0][0][1][1] - corners[0][0][0][1], 
                                   corners[0][0][1][0] - corners[0][0][0][0])
                
                return (x_pos, y_pos, math.degrees(heading))
        
        except Exception as e:
            print(f"❌ Vision positioning error: {e}")
        
        return None
    
    def cleanup(self):
        """Clean up camera resources"""
        if self.camera:
            try:
                self.camera.stop()
            except Exception:
                pass


class L298NMotorDriver:
    """L298N Motor Driver Interface for DC Motors with fixed GPIO pins"""
    
    def __init__(self, pwm_freq: int = 1000):
        """
        Initialize L298N motor driver with standardized pins
        
        Args:
            pwm_freq: PWM frequency in Hz
        """
        self.left_pins, self.right_pins = GPIOPinManager.get_motor_pins()
        self.pwm_freq = pwm_freq
        self.left_pwm = None
        self.right_pwm = None
        self.initialized = False
        
        if RPI_AVAILABLE:
            self.setup_gpio()
        else:
            print("🔧 Motor driver running in simulation mode")
    
    def setup_gpio(self):
        """Setup GPIO pins and PWM"""
        try:
            GPIO.setmode(GPIO.BCM)
            
            # Setup left motor pins
            GPIO.setup(self.left_pins['pwm'], GPIO.OUT)
            GPIO.setup(self.left_pins['in1'], GPIO.OUT)
            GPIO.setup(self.left_pins['in2'], GPIO.OUT)
            
            # Setup right motor pins
            GPIO.setup(self.right_pins['pwm'], GPIO.OUT)
            GPIO.setup(self.right_pins['in1'], GPIO.OUT)
            GPIO.setup(self.right_pins['in2'], GPIO.OUT)
            
            # Initialize PWM
            self.left_pwm = GPIO.PWM(self.left_pins['pwm'], self.pwm_freq)
            self.right_pwm = GPIO.PWM(self.right_pins['pwm'], self.pwm_freq)
            
            self.left_pwm.start(0)
            self.right_pwm.start(0)
            
            self.initialized = True
            print("🔧 L298N Motor Driver initialized:")
            GPIOPinManager.print_pin_assignment()
            
        except Exception as e:
            print(f"❌ Motor driver setup failed: {e}")
    
    def set_motor_speed(self, motor: str, speed: float, direction: str):
        """
        Control individual motor
        
        Args:
            motor: 'left' or 'right'
            speed: 0.0 to 1.0 (percentage)
            direction: 'forward', 'backward', or 'stop'
        """
        if not self.initialized and RPI_AVAILABLE:
            return
            
        speed = max(0.0, min(1.0, abs(speed)))  # Clamp to 0-1
        pwm_value = int(speed * 100)
        
        if motor == 'left':
            pins = self.left_pins
            pwm = self.left_pwm
        else:
            pins = self.right_pins  
            pwm = self.right_pwm
        
        if not RPI_AVAILABLE:
            print(f"🔧 {motor} motor: {direction} at {speed:.1%}")
            return
        
        # Set direction
        if direction == 'forward':
            GPIO.output(pins['in1'], GPIO.HIGH)
            GPIO.output(pins['in2'], GPIO.LOW)
        elif direction == 'backward':
            GPIO.output(pins['in1'], GPIO.LOW)
            GPIO.output(pins['in2'], GPIO.HIGH)
        else:  # stop
            GPIO.output(pins['in1'], GPIO.LOW)
            GPIO.output(pins['in2'], GPIO.LOW)
            pwm_value = 0
        
        # Set speed
        pwm.ChangeDutyCycle(pwm_value)
    
    def drive_tank(self, left_speed: float, right_speed: float):
        """
        Tank drive control
        
        Args:
            left_speed: -1.0 to 1.0 (negative = backward)
            right_speed: -1.0 to 1.0 (negative = backward)
        """
        # Left motor
        if left_speed > 0:
            self.set_motor_speed('left', left_speed, 'forward')
        elif left_speed < 0:
            self.set_motor_speed('left', abs(left_speed), 'backward')
        else:
            self.set_motor_speed('left', 0, 'stop')
        
        # Right motor  
        if right_speed > 0:
            self.set_motor_speed('right', right_speed, 'forward')
        elif right_speed < 0:
            self.set_motor_speed('right', abs(right_speed), 'backward')
        else:
            self.set_motor_speed('right', 0, 'stop')
    
    def drive_arcade(self, forward_speed: float, turn_rate: float):
        """
        Arcade drive control
        
        Args:
            forward_speed: -1.0 to 1.0 (forward/backward)
            turn_rate: -1.0 to 1.0 (left/right turn)
        """
        # Mix arcade controls into tank drives
        left_speed = forward_speed + turn_rate
        right_speed = forward_speed - turn_rate
        
        # Normalize if any speed exceeds ±1.0
        max_speed = max(abs(left_speed), abs(right_speed))
        if max_speed > 1.0:
            left_speed /= max_speed
            right_speed /= max_speed
        
        self.drive_tank(left_speed, right_speed)
    
    def stop(self):
        """Stop all motors"""
        self.drive_tank(0, 0)
    
    def cleanup(self):
        """Clean up GPIO resources"""
        if self.initialized:
            self.stop()
            if self.left_pwm:
                self.left_pwm.stop()
            if self.right_pwm:
                self.right_pwm.stop()


class ChipuRobot:
    """
    Unified ChipuRobo hardware interface
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
    
    def print_capabilities(self):
        """Print robot capabilities based on available hardware"""
        print("🔧 Robot Capabilities:")
        print(f"   ⚙️ Motor Control: {'✅ Active' if self.motor_driver.initialized or not RPI_AVAILABLE else '❌ Failed'}")
        print(f"   📏 Left Encoder: {'✅ Active' if self.left_encoder.active or not RPI_AVAILABLE else '❌ Failed'}")
        print(f"   📏 Right Encoder: {'✅ Active' if self.right_encoder.active or not RPI_AVAILABLE else '❌ Failed'}")
        print(f"   🧭 IMU: {'✅ Active' if self.imu.available or not RPI_AVAILABLE else '❌ Failed'}")
        print(f"   📷 Vision: {'✅ Active' if self.vision.available or not RPI_AVAILABLE else '❌ Failed'}")
    
    def start_position_tracking(self):
        """Start background thread for position tracking"""
        if self.position_thread and self.position_thread.is_alive():
            return
            
        self.running = True
        self.position_thread = threading.Thread(target=self._position_update_loop)
        self.position_thread.daemon = True
        self.position_thread.start()
        print("📍 Position tracking started")
    
    def stop_position_tracking(self):
        """Stop position tracking thread"""
        self.running = False
        if self.position_thread:
            self.position_thread.join(timeout=1.0)
        print("📍 Position tracking stopped")
    
    def _position_update_loop(self):
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
    
    def update_position_fusion(self, dt: float):
        """
        Update robot position using sensor fusion
        
        Args:
            dt: Time delta since last update
        """
        # 1. Get encoder-based movement
        left_dist = self.left_encoder.get_distance()  # in inches
        right_dist = self.right_encoder.get_distance()  # in inches
        
        # Calculate movement from encoders (differential drive)
        distance = (left_dist + right_dist) / 2.0
        delta_heading = (right_dist - left_dist) / self.config.get('wheelBase', 12.0)
        
        # 2. Get IMU heading correction
        imu_gyro = self.imu.get_gyro()
        imu_heading_rate = imu_gyro[2]  # Z-axis rotation in rad/s
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
                'left_distance': self.left_encoder.get_distance(),
                'right_distance': self.right_encoder.get_distance(),
                'left_velocity': self.left_encoder.get_velocity(),
                'right_velocity': self.right_encoder.get_velocity()
            },
            'imu': {
                'acceleration': self.imu.get_acceleration(),
                'gyro': self.imu.get_gyro(),
                'heading': self.imu.get_heading_degrees()
            },
            'vision': {
                'position': self.vision.get_position_from_markers()
            }
        }
    
    # Motor control methods (delegate to motor driver)
    def drive_tank(self, left_speed: float, right_speed: float):
        """Tank drive control (-1.0 to 1.0 for each side)"""
        self.motor_driver.drive_tank(left_speed, right_speed)
    
    def drive_arcade(self, forward_speed: float, turn_rate: float):
        """Arcade drive control (-1.0 to 1.0 for forward and turn)"""
        self.motor_driver.drive_arcade(forward_speed, turn_rate)
    
    def stop(self):
        """Stop all motors"""
        self.motor_driver.stop()
    
    def execute_mission(self, mission_data: Dict[str, Any]):
        """
        Execute a mission from the web interface
        
        Args:
            mission_data: Mission configuration from robot_server.py
        """
        print(f"🎯 Executing mission: {mission_data.get('name', 'Unnamed')}")
        
        waypoints = mission_data.get('waypoints', [])
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
    
    def move_to_waypoint(self, waypoint: Dict[str, float]):
        """
        Move to a specific waypoint using position feedback
        
        Args:
            waypoint: Dictionary with 'x' and 'y' coordinates
        """
        target_x = waypoint['x']
        target_y = waypoint['y']
        tolerance = 2.0 / 12  # feet (2 inches)
        
        max_speed = 0.5  # Max motor power ratio (0.0–1.0), not physical speed
        
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
    
    def reset_position(self):
        """Reset robot position to origin"""
        self.position = {'x': 0.0, 'y': 0.0, 'heading': 0.0}
        self.left_encoder.reset()
        self.right_encoder.reset()
        print("📍 Position reset to origin")
    
    def cleanup(self):
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


def main():
    """Test the unified robot system"""
    print("🤖 ChipuRobot Test Program")
    
    # Test configuration
    config = {
        'wheelDiameter': 4.0,    # inches
        'wheelBase': 12.0,       # inches
        'encoderPPR': 11,        # pulses per revolution
        'pwmFreq': 1000          # Hz
    }
    
    # Create robot instance
    robot = ChipuRobot(config)
    
    try:
        # Test basic movements
        print("\n🧪 Testing basic movements...")
        
        # Forward
        print("   Moving forward...")
        robot.drive_arcade(0.3, 0)
        time.sleep(2)
        
        # Turn right
        print("   Turning right...")
        robot.drive_arcade(0, 0.5)
        time.sleep(1)
        
        # Backward
        print("   Moving backward...")
        robot.drive_arcade(-0.3, 0)
        time.sleep(2)
        
        # Stop
        robot.stop()
        
        # Test sensor readings
        print("\n📊 Sensor readings:")
        sensor_data = robot.get_sensor_data()
        print(f"   Position: {sensor_data['position']}")
        print(f"   Encoders: {sensor_data['encoders']}")
        print(f"   IMU: {sensor_data['imu']}")
        
    except KeyboardInterrupt:
        print("\n⏹️ Test interrupted by user")
    
    finally:
        robot.cleanup()


if __name__ == '__main__':
    main()