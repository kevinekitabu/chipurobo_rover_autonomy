#!/usr/bin/env python3
"""
Robot with IMU + Encoders for Better Precision
"""

# Hardware needed:
# - MPU6050 or BNO055 IMU sensor
# - I2C connection to Pi

class IMUEnhancedRobot:
    """Robot with IMU for accurate heading measurement"""
    
    def __init__(self, config):
        self.config = config
        
        # IMU for heading (compass/gyroscope)
        try:
            import board
            import adafruit_mpu6050
            i2c = board.I2C()
            self.imu = adafruit_mpu6050.MPU6050(i2c)
            self.imu_available = True
            print("✅ IMU initialized - accurate heading available")
        except ImportError:
            print("⚠️ IMU not available - install adafruit-circuitpython-mpu6050")
            self.imu_available = False
        
        # Encoders for distance
        self.left_encoder = WheelEncoder(pin_a=5, pin_b=6)
        self.right_encoder = WheelEncoder(pin_a=13, pin_b=19)
        
        # Position with IMU-corrected heading
        self.position = {'x': 0.0, 'y': 0.0, 'heading': 0.0}
        self.heading_offset = 0.0  # Calibration offset
    
    def get_imu_heading(self):
        """Get accurate heading from IMU"""
        if not self.imu_available:
            return None
            
        # Read gyroscope Z-axis for rotation
        gyro_z = self.imu.gyro[2]  # radians/second
        accel_x, accel_y, accel_z = self.imu.acceleration
        
        # Calculate heading from magnetometer (if available) or integrate gyro
        # This is simplified - real implementation would use sensor fusion
        heading = math.atan2(accel_y, accel_x) * 180 / math.pi
        return (heading + self.heading_offset) % 360
    
    def update_position_with_imu(self):
        """Update position using encoders + IMU heading"""
        # Get distances from encoders
        wheel_diameter = self.config['wheelDiameter']
        left_distance = self.left_encoder.get_distance(wheel_diameter)
        right_distance = self.right_encoder.get_distance(wheel_diameter)
        distance_traveled = (left_distance + right_distance) / 2.0
        
        # Get accurate heading from IMU
        if self.imu_available:
            self.position['heading'] = self.get_imu_heading()
        else:
            # Fallback to encoder-based heading
            wheelbase = self.config['wheelbase'] 
            heading_change = (right_distance - left_distance) / wheelbase
            self.position['heading'] += heading_change * 180 / math.pi
        
        # Calculate movement in X,Y
        heading_rad = math.radians(self.position['heading'])
        dx = distance_traveled * math.cos(heading_rad) / 12.0  # inches to feet
        dy = distance_traveled * math.sin(heading_rad) / 12.0
        
        self.position['x'] += dx
        self.position['y'] += dy
        
        # Reset encoders
        self.left_encoder.reset()
        self.right_encoder.reset()
        
        return self.position

# Hardware Shopping List for IMU:
# - MPU6050 IMU (about $3-5)
# - Or BNO055 9-DOF sensor (more accurate, ~$15-20)
# - I2C jumper wires