#!/usr/bin/env python3
"""
Raspberry Pi Robot Hardware Interface
Controls DC motors via L298N motor driver for ChipuRobo
"""

try:
    import RPi.GPIO as GPIO
    import time
    import math
    RPI_AVAILABLE = True
except ImportError:
    print("⚠️ RPi.GPIO not available - running in simulation mode")
    RPI_AVAILABLE = False

class L298NMotorDriver:
    """L298N Motor Driver Interface for DC Motors"""
    
    def __init__(self, left_pins=None, right_pins=None, pwm_freq=1000):
        """
        Initialize L298N motor driver
        
        Args:
            left_pins: dict with keys 'pwm', 'in1', 'in2' for left motor
            right_pins: dict with keys 'pwm', 'in1', 'in2' for right motor
            pwm_freq: PWM frequency in Hz
        """
        # Default pin configuration for L298N
        self.left_pins = left_pins or {
            'pwm': 18,  # ENA pin (PWM)
            'in1': 24,  # IN1 pin
            'in2': 23   # IN2 pin
        }
        
        self.right_pins = right_pins or {
            'pwm': 19,  # ENB pin (PWM)
            'in1': 21,  # IN3 pin
            'in2': 20   # IN4 pin
        }
        
        self.pwm_freq = pwm_freq
        self.left_pwm = None
        self.right_pwm = None
        self.initialized = False
        
        if RPI_AVAILABLE:
            self.setup_gpio()
        else:
            print("🔧 Motor driver running in simulation mode")
    
    def setup_gpio(self):
        """Setup GPIO pins for L298N"""
        try:
            GPIO.setmode(GPIO.BCM)
            GPIO.setwarnings(False)
            
            # Setup left motor pins
            GPIO.setup(self.left_pins['pwm'], GPIO.OUT)
            GPIO.setup(self.left_pins['in1'], GPIO.OUT)
            GPIO.setup(self.left_pins['in2'], GPIO.OUT)
            
            # Setup right motor pins  
            GPIO.setup(self.right_pins['pwm'], GPIO.OUT)
            GPIO.setup(self.right_pins['in1'], GPIO.OUT)
            GPIO.setup(self.right_pins['in2'], GPIO.OUT)
            
            # Setup PWM
            self.left_pwm = GPIO.PWM(self.left_pins['pwm'], self.pwm_freq)
            self.right_pwm = GPIO.PWM(self.right_pins['pwm'], self.pwm_freq)
            
            self.left_pwm.start(0)
            self.right_pwm.start(0)
            
            self.initialized = True
            print("✅ L298N motor driver initialized successfully")
            print(f"   Left motor: PWM={self.left_pins['pwm']}, IN1={self.left_pins['in1']}, IN2={self.left_pins['in2']}")
            print(f"   Right motor: PWM={self.right_pins['pwm']}, IN1={self.right_pins['in1']}, IN2={self.right_pins['in2']}")
            print(f"   PWM Frequency: {self.pwm_freq} Hz")
            
        except Exception as e:
            print(f"❌ Failed to initialize GPIO: {e}")
            self.initialized = False
    
    def set_motor_speed(self, left_speed, right_speed):
        """
        Set motor speeds
        
        Args:
            left_speed: Speed from -100 to 100 (negative = reverse)
            right_speed: Speed from -100 to 100 (negative = reverse)
        """
        if not RPI_AVAILABLE:
            print(f"🎮 SIMULATION: Left={left_speed:.1f}%, Right={right_speed:.1f}%")
            return
        
        if not self.initialized:
            print("❌ Motor driver not initialized")
            return
        
        # Clamp speeds to valid range
        left_speed = max(-100, min(100, left_speed))
        right_speed = max(-100, min(100, right_speed))
        
        # Set left motor
        self._set_single_motor(
            abs(left_speed),
            left_speed >= 0,
            self.left_pwm,
            self.left_pins['in1'],
            self.left_pins['in2']
        )
        
        # Set right motor
        self._set_single_motor(
            abs(right_speed),
            right_speed >= 0,
            self.right_pwm,
            self.right_pins['in1'],
            self.right_pins['in2']
        )
    
    def _set_single_motor(self, speed, forward, pwm, in1_pin, in2_pin):
        """Set individual motor speed and direction"""
        if forward:
            GPIO.output(in1_pin, GPIO.HIGH)
            GPIO.output(in2_pin, GPIO.LOW)
        else:
            GPIO.output(in1_pin, GPIO.LOW)
            GPIO.output(in2_pin, GPIO.HIGH)
        
        pwm.ChangeDutyCycle(speed)
    
    def stop_motors(self):
        """Stop both motors"""
        self.set_motor_speed(0, 0)
    
    def cleanup(self):
        """Cleanup GPIO resources"""
        if RPI_AVAILABLE and self.initialized:
            self.stop_motors()
            if self.left_pwm:
                self.left_pwm.stop()
            if self.right_pwm:
                self.right_pwm.stop()
            GPIO.cleanup()
            print("✅ Motor driver cleanup complete")

class RaspberryPiRobot:
    """Main robot control class for Raspberry Pi with DC motors"""
    
    def __init__(self, config=None):
        """
        Initialize Raspberry Pi robot
        
        Args:
            config: Robot configuration dict with wheel diameter, wheelbase, etc.
        """
        self.config = config or {
            'wheelDiameter': 4.0,  # inches
            'wheelbase': 8.0,      # inches
            'maxSpeed': 3.0,       # ft/s
            'maxAccel': 2.0,       # ft/s²
            'pwmFreq': 1000        # Hz
        }
        
        # Initialize motor driver
        self.motor_driver = L298NMotorDriver(pwm_freq=self.config.get('pwmFreq', 1000))
        
        # Robot state
        self.current_position = {'x': 0.0, 'y': 0.0, 'heading': 0.0}
        self.current_velocity = {'linear': 0.0, 'angular': 0.0}
        
        # Calculate wheel circumference (inches)
        self.wheel_circumference = math.pi * self.config['wheelDiameter']
        
        # Initialize precision sensors if available
        self.precision_sensors = self.setup_precision_sensors()
        
        print("🤖 Raspberry Pi robot initialized")
        print(f"   Wheel diameter: {self.config['wheelDiameter']} inches")
        print(f"   Wheelbase: {self.config['wheelbase']} inches")
        print(f"   Max speed: {self.config['maxSpeed']} ft/s")
        if self.precision_sensors:
            print("   🎯 High-precision mode: encoders + IMU + camera available")
        else:
            print("   📍 Basic mode: precision sensors not available")
    
    def setup_precision_sensors(self):
        """Try to initialize precision sensors (encoders, IMU, camera)"""
        if not RPI_AVAILABLE:
            return False
            
        try:
            # Check if precision sensors are available
            precision_available = True
            
            # Try to import precision modules
            try:
                import board
                import adafruit_mpu6050
                import cv2
                import picamera2
                precision_available = True
            except ImportError as e:
                print(f"⚠️ Precision sensor libraries not available: {e}")
                print("   To enable precision mode, install:")
                print("   pip3 install adafruit-circuitpython-mpu6050 opencv-python picamera2")
                return False
            
            if precision_available:
                # Initialize the high-precision robot components
                self.encoders = {
                    'left': {'pin_a': 5, 'pin_b': 6, 'count': 0},
                    'right': {'pin_a': 13, 'pin_b': 19, 'count': 0}
                }
                self.imu_available = True
                self.camera_available = True
                return True
        
        except Exception as e:
            print(f"⚠️ Precision sensors initialization failed: {e}")
        
        return False
    
    def drive_tank(self, left_speed, right_speed):
        """
        Tank drive control
        
        Args:
            left_speed: Left wheel speed (-100 to 100)
            right_speed: Right wheel speed (-100 to 100) 
        """
        self.motor_driver.set_motor_speed(left_speed, right_speed)
    
    def drive_arcade(self, forward_speed, turn_rate):
        """
        Arcade drive control
        
        Args:
            forward_speed: Forward/backward speed (-100 to 100)
            turn_rate: Turn rate (-100 to 100, negative = left)
        """
        # Convert arcade to tank drive
        left_speed = forward_speed + turn_rate
        right_speed = forward_speed - turn_rate
        
        # Normalize if values exceed limits
        max_val = max(abs(left_speed), abs(right_speed))
        if max_val > 100:
            left_speed = (left_speed / max_val) * 100
            right_speed = (right_speed / max_val) * 100
        
        self.drive_tank(left_speed, right_speed)
    
    def drive_to_position(self, target_x, target_y, timeout=30.0):
        """
        Drive to a specific field position
        
        Args:
            target_x: Target X coordinate (feet)
            target_y: Target Y coordinate (feet)
            timeout: Maximum time to reach target (seconds)
        """
        print(f"🎯 Driving to position ({target_x:.1f}ft, {target_y:.1f}ft)")
        
        start_time = time.time()
        tolerance = 0.2  # feet
        
        while time.time() - start_time < timeout:
            # Calculate distance and angle to target
            dx = target_x - self.current_position['x']
            dy = target_y - self.current_position['y']
            distance = math.sqrt(dx*dx + dy*dy)
            
            if distance < tolerance:
                print("✅ Target reached!")
                self.stop()
                return True
            
            # Calculate heading to target
            target_heading = math.atan2(dy, dx) * 180 / math.pi
            heading_error = target_heading - self.current_position['heading']
            
            # Normalize heading error to [-180, 180]
            while heading_error > 180:
                heading_error -= 360
            while heading_error < -180:
                heading_error += 360
            
            # Simple proportional control
            forward_speed = min(distance * 20, 60)  # Scale distance to speed
            turn_rate = heading_error * 0.8  # Proportional turning
            
            self.drive_arcade(forward_speed, turn_rate)
            
            # Simulate position update (in real robot, use odometry/sensors)
            if not RPI_AVAILABLE:
                # Simple simulation - assume robot moves toward target
                move_speed = 0.1  # ft per iteration
                if distance > 0:
                    self.current_position['x'] += (dx / distance) * move_speed
                    self.current_position['y'] += (dy / distance) * move_speed
                    self.current_position['heading'] = target_heading
            
            time.sleep(0.1)
        
        print("⚠️ Drive timeout reached")
        self.stop()
        return False
    
    def execute_trajectory(self, trajectory):
        """
        Execute a trajectory of waypoints
        
        Args:
            trajectory: List of waypoint dicts with x, y, time, velocity, heading
        """
        print(f"🚀 Executing trajectory with {len(trajectory)} points")
        
        for i, point in enumerate(trajectory):
            print(f"   Waypoint {i+1}/{len(trajectory)}: ({point['x']:.1f}ft, {point['y']:.1f}ft)")
            
            success = self.drive_to_position(point['x'], point['y'])
            if not success:
                print(f"❌ Failed to reach waypoint {i+1}")
                return False
            
            # Brief pause at each waypoint
            time.sleep(0.5)
        
        print("✅ Trajectory execution complete!")
        return True
    
    def stop(self):
        """Stop the robot"""
        self.motor_driver.stop_motors()
        self.current_velocity = {'linear': 0.0, 'angular': 0.0}
    
    def get_status(self):
        """Get current robot status"""
        return {
            'position': self.current_position,
            'velocity': self.current_velocity,
            'config': self.config,
            'hardware': 'raspberry_pi_l298n',
            'initialized': self.motor_driver.initialized
        }
    
    def cleanup(self):
        """Cleanup robot resources"""
        self.stop()
        self.motor_driver.cleanup()

# Test functions
def test_motors():
    """Test motor functionality"""
    print("🧪 Testing Raspberry Pi motor control...")
    
    robot = RaspberryPiRobot()
    
    print("Testing forward movement...")
    robot.drive_arcade(50, 0)  # 50% forward
    time.sleep(2)
    
    print("Testing turn...")
    robot.drive_arcade(0, 50)  # Turn right
    time.sleep(2)
    
    print("Testing reverse...")
    robot.drive_arcade(-30, 0)  # 30% reverse
    time.sleep(2)
    
    print("Stopping...")
    robot.stop()
    
    robot.cleanup()
    print("✅ Motor test complete")

if __name__ == '__main__':
    # Run motor test if script is executed directly
    test_motors()