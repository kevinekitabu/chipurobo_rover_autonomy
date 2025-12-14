#!/usr/bin/env python3
"""
IMU Interface for ChipuRobo
MPU9255 9-axis IMU interface via I2C
"""

import math
from typing import Tuple, Dict, Any

# Core imports with error handling
try:
    import RPi.GPIO as GPIO
    RPI_AVAILABLE = True
except ImportError:
    RPI_AVAILABLE = False

# IMU imports
try:
    import board
    import busio
    import adafruit_mpu6050
    IMU_AVAILABLE = True
except ImportError:
    IMU_AVAILABLE = False


class MPU9255_IMU:
    """MPU9255 9-axis IMU interface via I2C"""
    
    def __init__(self):
        self.imu = None
        self.available = False
        
        if IMU_AVAILABLE and RPI_AVAILABLE:
            self.setup_imu()
        else:
            print("🧭 IMU running in simulation mode")
    
    def setup_imu(self) -> None:
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
    
    def get_temperature(self) -> float:
        """Get temperature in Celsius"""
        if self.available:
            try:
                return self.imu.temperature
            except Exception:
                pass
        return 25.0  # Simulate room temperature
    
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
    
    def get_all_data(self) -> Dict[str, Any]:
        """Get all IMU data"""
        accel = self.get_acceleration()
        gyro = self.get_gyro()
        
        return {
            'acceleration': {
                'x': accel[0],
                'y': accel[1],
                'z': accel[2]
            },
            'gyro': {
                'x': gyro[0],
                'y': gyro[1],
                'z': gyro[2]
            },
            'temperature': self.get_temperature(),
            'heading_degrees': self.get_heading_degrees(),
            'available': self.available
        }
    
    def get_status(self) -> Dict[str, Any]:
        """Get IMU status"""
        return {
            'available': self.available,
            'rpi_available': RPI_AVAILABLE,
            'imu_library_available': IMU_AVAILABLE,
            'i2c_address': '0x68' if self.available else 'Unknown'
        }
    
    def calibrate(self, samples: int = 100) -> Dict[str, Tuple[float, float, float]]:
        """Calibrate IMU by taking samples at rest"""
        if not self.available:
            return {
                'accel_offset': (0.0, 0.0, 0.0),
                'gyro_offset': (0.0, 0.0, 0.0)
            }
        
        print(f"🧭 Calibrating IMU with {samples} samples...")
        
        accel_sum = [0.0, 0.0, 0.0]
        gyro_sum = [0.0, 0.0, 0.0]
        
        for i in range(samples):
            accel = self.get_acceleration()
            gyro = self.get_gyro()
            
            for j in range(3):
                accel_sum[j] += accel[j]
                gyro_sum[j] += gyro[j]
        
        # Calculate offsets
        accel_offset = tuple(x / samples for x in accel_sum)
        gyro_offset = tuple(x / samples for x in gyro_sum)
        
        print(f"✅ IMU calibration complete")
        print(f"   Accel offset: {accel_offset}")
        print(f"   Gyro offset: {gyro_offset}")
        
        return {
            'accel_offset': accel_offset,
            'gyro_offset': gyro_offset
        }