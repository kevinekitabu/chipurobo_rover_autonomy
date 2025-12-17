
#!/usr/bin/env python3
"""
ChipuRobo v0.5 - Motor Control for Computer Vision Rover
2-channel differential drive control for basic autonomous movement
"""

import time
from typing import Optional

# Hardware imports with fallback for development
try:
    from gpiozero import Motor, PWMOutputDevice
    HARDWARE_AVAILABLE = True
except ImportError:
    HARDWARE_AVAILABLE = False
    print("🔧 Running in simulation mode - no GPIO hardware")


class MotorController:
    """
    Simple differential drive motor controller for vision-based rover
    Supports basic movement primitives needed for autonomous behavior
    """
    
    def __init__(self, speed: float = 0.8):
        """
        Initialize motor controller for differential drive
        
        Args:
            speed: Default speed for movements (0.0 - 1.0)
        """
        self.speed = speed
        self.is_moving = False
        self.current_action = "stopped"
        
        if HARDWARE_AVAILABLE:
            # Hardware setup for ChipuRobo v0.5
            # Left motors (wired together)
            self.motor_left = Motor(forward=17, backward=27)
            # Right motors (wired together)  
            self.motor_right = Motor(forward=22, backward=23)
            
            # Enable pins for motor driver
            self.enable_left = PWMOutputDevice(24)
            self.enable_right = PWMOutputDevice(25)
            
            # Set enables HIGH for full power
            self.enable_left.value = 1.0
            self.enable_right.value = 1.0
            
            print("🚗 Motor controller initialized - hardware mode")
        else:
            print("🚗 Motor controller initialized - simulation mode")
    
    def stop(self):
        """Stop all motors immediately"""
        if HARDWARE_AVAILABLE:
            self.motor_left.stop()
            self.motor_right.stop()
        
        self.is_moving = False
        self.current_action = "stopped"
        print("🛑 STOP")
    
    def forward(self, duration: Optional[float] = None):
        """
        Move forward - both motors same direction
        
        Args:
            duration: Optional time in seconds to move forward
        """
        if HARDWARE_AVAILABLE:
            self.motor_left.forward(self.speed)
            self.motor_right.forward(self.speed)
        
        self.is_moving = True
        self.current_action = "forward"
        print("⬆️ FORWARD")
        
        if duration:
            time.sleep(duration)
            self.stop()
    
    def turn_left(self, duration: float = 0.5):
        """
        Turn left - left motors backward, right motors forward
        
        Args:
            duration: Time in seconds to turn (default 0.5s for ~90 degrees)
        """
        if HARDWARE_AVAILABLE:
            self.motor_left.backward(self.speed)
            self.motor_right.forward(self.speed)
        
        self.is_moving = True
        self.current_action = "turning_left"
        print("↪️ TURN LEFT")
        
        # Time-based turning (no encoders)
        time.sleep(duration)
        self.stop()
    
    def turn_right(self, duration: float = 0.5):
        """
        Turn right - left motors forward, right motors backward
        
        Args:
            duration: Time in seconds to turn (default 0.5s for ~90 degrees)
        """
        if HARDWARE_AVAILABLE:
            self.motor_left.forward(self.speed)
            self.motor_right.backward(self.speed)
        
        self.is_moving = True
        self.current_action = "turning_right"
        print("↩️ TURN RIGHT")
        
        # Time-based turning (no encoders)
        time.sleep(duration)
        self.stop()
    
    def set_speed(self, speed: float):
        """
        Set motor speed for all movements
        
        Args:
            speed: Speed value between 0.0 and 1.0
        """
        if 0.0 <= speed <= 1.0:
            self.speed = speed
            print(f"🎛️ Speed set to {speed:.1%}")
        else:
            print("⚠️ Speed must be between 0.0 and 1.0")
    
    def get_status(self) -> dict:
        """Get current motor status"""
        return {
            "is_moving": self.is_moving,
            "current_action": self.current_action,
            "speed": self.speed,
            "hardware_available": HARDWARE_AVAILABLE
        }


# Legacy function support for backward compatibility
def stop():
    """Legacy function - use MotorController.stop() instead"""
    if hasattr(stop, '_controller'):
        stop._controller.stop()

def forward():
    """Legacy function - use MotorController.forward() instead"""
    if hasattr(forward, '_controller'):
        forward._controller.forward()

def left():
    """Legacy function - use MotorController.turn_left() instead"""
    if hasattr(left, '_controller'):
        left._controller.turn_left()

def right():
    """Legacy function - use MotorController.turn_right() instead"""
    if hasattr(right, '_controller'):
        right._controller.turn_right()

# Create global controller instance for legacy support
if HARDWARE_AVAILABLE:
    _global_controller = MotorController()
    stop._controller = _global_controller
    forward._controller = _global_controller
    left._controller = _global_controller
    right._controller = _global_controller