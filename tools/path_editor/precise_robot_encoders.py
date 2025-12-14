#!/usr/bin/env python3
"""
Enhanced Robot with Wheel Encoders for Precise Movement
"""

import RPi.GPIO as GPIO
import time
import math

class WheelEncoder:
    """Wheel encoder for precise distance/speed measurement"""
    
    def __init__(self, pin_a, pin_b, pulses_per_revolution=20):
        self.pin_a = pin_a
        self.pin_b = pin_b
        self.ppr = pulses_per_revolution
        self.count = 0
        self.last_time = time.time()
        self.rpm = 0.0
        
        # Setup GPIO pins
        GPIO.setup(pin_a, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(pin_b, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        
        # Add interrupt for encoder pulses
        GPIO.add_event_detect(pin_a, GPIO.RISING, callback=self._encoder_callback)
    
    def _encoder_callback(self, channel):
        """Handle encoder pulse - called on interrupt"""
        current_time = time.time()
        dt = current_time - self.last_time
        
        # Read direction from pin B
        if GPIO.input(self.pin_b):
            self.count += 1  # Forward
        else:
            self.count -= 1  # Backward
            
        # Calculate RPM
        if dt > 0:
            self.rpm = 60.0 / (dt * self.ppr)  # RPM calculation
        self.last_time = current_time
    
    def get_distance(self, wheel_diameter):
        """Get distance traveled in inches"""
        wheel_circumference = math.pi * wheel_diameter
        revolutions = self.count / self.ppr
        return revolutions * wheel_circumference
    
    def reset(self):
        """Reset encoder count"""
        self.count = 0

class PreciseRaspberryPiRobot:
    """Robot with encoder feedback for precise positioning"""
    
    def __init__(self, config):
        self.config = config
        
        # Initialize encoders
        self.left_encoder = WheelEncoder(pin_a=5, pin_b=6)   # Left wheel encoder
        self.right_encoder = WheelEncoder(pin_a=13, pin_b=19) # Right wheel encoder
        
        # Motor driver (same as before)
        self.motor_driver = L298NMotorDriver()
        
        # Position tracking with encoder feedback
        self.position = {'x': 0.0, 'y': 0.0, 'heading': 0.0}
        
    def update_position(self):
        """Update robot position using wheel encoders (dead reckoning)"""
        wheel_diameter = self.config['wheelDiameter']
        wheelbase = self.config['wheelbase']
        
        # Get distances from encoders
        left_distance = self.left_encoder.get_distance(wheel_diameter)
        right_distance = self.right_encoder.get_distance(wheel_diameter)
        
        # Calculate robot movement
        distance_traveled = (left_distance + right_distance) / 2.0
        heading_change = (right_distance - left_distance) / wheelbase
        
        # Update position
        self.position['heading'] += heading_change * 180 / math.pi
        
        # Convert to X,Y movement
        dx = distance_traveled * math.cos(math.radians(self.position['heading']))
        dy = distance_traveled * math.sin(math.radians(self.position['heading']))
        
        self.position['x'] += dx / 12.0  # Convert inches to feet
        self.position['y'] += dy / 12.0
        
        # Reset encoders for next iteration
        self.left_encoder.reset()
        self.right_encoder.reset()
        
        return self.position
    
    def drive_to_position_precise(self, target_x, target_y, timeout=30):
        """Drive to position with encoder feedback"""
        print(f"🎯 Driving precisely to ({target_x:.1f}ft, {target_y:.1f}ft)")
        
        start_time = time.time()
        tolerance = 0.1  # Much tighter tolerance with encoders
        
        while time.time() - start_time < timeout:
            # Update position from encoders
            current_pos = self.update_position()
            
            # Calculate error
            dx = target_x - current_pos['x']
            dy = target_y - current_pos['y']
            distance = math.sqrt(dx*dx + dy*dy)
            
            if distance < tolerance:
                print(f"✅ Precise target reached! Final error: {distance:.2f}ft")
                self.motor_driver.stop_motors()
                return True
            
            # PID-style control with encoder feedback
            target_heading = math.atan2(dy, dx) * 180 / math.pi
            heading_error = target_heading - current_pos['heading']
            
            # Normalize heading
            while heading_error > 180: heading_error -= 360
            while heading_error < -180: heading_error += 360
            
            # More precise control with encoder feedback
            forward_speed = min(distance * 30, 80)  # Proportional to distance
            turn_rate = heading_error * 1.2  # Proportional turning
            
            self.drive_arcade(forward_speed, turn_rate)
            time.sleep(0.05)  # Fast update rate with encoders
        
        print("❌ Timeout - target not reached precisely")
        self.motor_driver.stop_motors()
        return False

# Hardware Shopping List for Encoders:
# - 2x Rotary Encoders (e.g., KY-040 or optical wheel encoders)
# - Mounting brackets to attach to wheels/motors
# - Encoder wheels/discs if using optical encoders