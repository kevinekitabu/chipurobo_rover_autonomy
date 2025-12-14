#!/usr/bin/env python3
"""
GPIO Pin Manager for ChipuRobo
Centralized GPIO pin assignment manager to prevent conflicts
"""

from typing import Dict, Tuple, Any


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
    def get_motor_pins(cls) -> Tuple[Dict[str, int], Dict[str, int]]:
        """Get motor pin configurations"""
        return cls.PINS['left_motor'], cls.PINS['right_motor']
    
    @classmethod
    def get_encoder_pins(cls) -> Tuple[Dict[str, int], Dict[str, int]]:
        """Get encoder pin configurations"""
        return cls.PINS['left_encoder'], cls.PINS['right_encoder']
    
    @classmethod
    def get_all_gpio_pins(cls) -> list:
        """Get all GPIO pins in use"""
        pins = []
        for component in ['left_motor', 'right_motor', 'left_encoder', 'right_encoder']:
            if component in cls.PINS:
                for pin_name, pin_num in cls.PINS[component].items():
                    if isinstance(pin_num, int):
                        pins.append(pin_num)
        
        # Add other single pins
        if isinstance(cls.PINS.get('camera_servo'), int):
            pins.append(cls.PINS['camera_servo'])
        if isinstance(cls.PINS.get('status_led'), int):
            pins.append(cls.PINS['status_led'])
            
        return sorted(pins)
    
    @classmethod
    def validate_pin_assignments(cls) -> bool:
        """Validate that no pins are assigned to multiple components"""
        all_pins = cls.get_all_gpio_pins()
        return len(all_pins) == len(set(all_pins))
    
    @classmethod
    def print_pin_assignment(cls) -> None:
        """Print formatted pin assignments"""
        print("🔌 GPIO Pin Assignment:")
        print(f"   Left Motor:  PWM={cls.PINS['left_motor']['pwm']}, IN1={cls.PINS['left_motor']['in1']}, IN2={cls.PINS['left_motor']['in2']}")
        print(f"   Right Motor: PWM={cls.PINS['right_motor']['pwm']}, IN1={cls.PINS['right_motor']['in1']}, IN2={cls.PINS['right_motor']['in2']}")
        print(f"   Left Encoder: A={cls.PINS['left_encoder']['channel_a']}, B={cls.PINS['left_encoder']['channel_b']}")
        print(f"   Right Encoder: A={cls.PINS['right_encoder']['channel_a']}, B={cls.PINS['right_encoder']['channel_b']}")
        print(f"   IMU: I2C SDA={cls.PINS['imu']['sda']}, SCL={cls.PINS['imu']['scl']}")
        print(f"   Camera: {cls.PINS['camera']} interface")
        
        # Validation
        if cls.validate_pin_assignments():
            print("✅ No pin conflicts detected")
        else:
            print("⚠️ Pin conflicts detected!")
    
    @classmethod
    def get_pin_usage_summary(cls) -> Dict[str, Any]:
        """Get summary of pin usage for diagnostics"""
        return {
            'total_gpio_pins': len(cls.get_all_gpio_pins()),
            'motor_pins': 6,  # 3 per motor
            'encoder_pins': 4,  # 2 per encoder
            'i2c_pins': 2,
            'other_pins': len([p for p in [cls.PINS.get('camera_servo'), cls.PINS.get('status_led')] if isinstance(p, int)]),
            'conflicts': not cls.validate_pin_assignments(),
            'all_pins': cls.get_all_gpio_pins()
        }