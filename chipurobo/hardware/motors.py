#!/usr/bin/env python3
"""
L298N Motor Driver for ChipuRobo
Professional motor control interface for DC motors
"""

from typing import Dict, Any
from .gpio_manager import GPIOPinManager

# Core imports with error handling
try:
    import RPi.GPIO as GPIO
    RPI_AVAILABLE = True
except ImportError:
    RPI_AVAILABLE = False


class L298NMotorDriver:
    """L298N Motor Driver Interface for DC Motors with standardized GPIO pins"""
    
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
    
    def setup_gpio(self) -> None:
        """Setup GPIO pins and PWM"""
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
    
    def set_motor_speed(self, motor: str, speed: float, direction: str) -> None:
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
        if pwm:
            pwm.ChangeDutyCycle(pwm_value)
    
    def drive_tank(self, left_speed: float, right_speed: float) -> None:
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
    
    def drive_arcade(self, forward_speed: float, turn_rate: float) -> None:
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
    
    def stop(self) -> None:
        """Stop all motors"""
        self.drive_tank(0, 0)
    
    def get_status(self) -> Dict[str, Any]:
        """Get motor driver status"""
        return {
            'initialized': self.initialized,
            'rpi_available': RPI_AVAILABLE,
            'pwm_frequency': self.pwm_freq,
            'pin_assignments': {
                'left_motor': self.left_pins,
                'right_motor': self.right_pins
            }
        }
    
    def cleanup(self) -> None:
        """Clean up GPIO resources"""
        if self.initialized:
            self.stop()
            if self.left_pwm:
                self.left_pwm.stop()
            if self.right_pwm:
                self.right_pwm.stop()
            print("🔧 Motor driver cleanup completed")