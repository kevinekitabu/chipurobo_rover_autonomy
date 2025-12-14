#!/usr/bin/env python3
"""
CORRECT Hardware Setup for ChipuRobo
DC Motors with Built-in Encoders + MPU9255 IMU + Pi Camera
"""

# 🔧 CORRECT HARDWARE SPECIFICATION:

correct_hardware = """
ACTUAL HARDWARE YOU NEED:

1. DC MOTORS WITH BUILT-IN ENCODERS:
   ✅ DC Geared Motors with Hall Effect Encoders
   ✅ Typically 11-40 PPR (Pulses Per Revolution)
   ✅ 6-wire connection: Motor (2) + Encoder (4)
   ✅ Examples:
      - JGA25-371 Motors with encoders
      - TT Motors with encoder wheels
      - Pololu micro metal geared motors with encoders
   
   WIRING:
   Motor: Red/Black → L298N motor outputs
   Encoder: VCC(3.3V), GND, Channel A, Channel B → Pi GPIO

2. IMU - GY-9255 (MPU9255):
   ✅ 9-axis sensor (gyro + accel + magnetometer)  
   ✅ Much better than MPU6050!
   ✅ I2C interface
   
   WIRING:
   VCC → Pi 3.3V
   GND → Pi GND  
   SDA → Pi GPIO 2
   SCL → Pi GPIO 3

3. RASPBERRY PI AI CAMERA:
   ✅ You already have this!
"""

# 🔌 CORRECT WIRING DIAGRAM:

correct_wiring = """
CORRECT WIRING FOR MOTOR ENCODERS:

DC MOTOR WITH BUILT-IN ENCODER (6-wire):
==========================================
Left Motor:
- Red wire    → L298N OUT1
- Black wire  → L298N OUT2  
- Encoder VCC → Pi 3.3V (Pin 1)
- Encoder GND → Pi GND (Pin 6)
- Encoder Ch A → Pi GPIO 5 (Pin 29)
- Encoder Ch B → Pi GPIO 6 (Pin 31)

Right Motor: 
- Red wire    → L298N OUT3
- Black wire  → L298N OUT4
- Encoder VCC → Pi 3.3V (Pin 1)  
- Encoder GND → Pi GND (Pin 6)
- Encoder Ch A → Pi GPIO 13 (Pin 33)
- Encoder Ch B → Pi GPIO 19 (Pin 35)

GY-9255 IMU:
============
- VCC → Pi 3.3V (Pin 1)
- GND → Pi GND (Pin 6)
- SDA → Pi GPIO 2 (Pin 3) 
- SCL → Pi GPIO 3 (Pin 5)

POWER NOTES:
- Motor power: 6-12V from battery → L298N
- Logic power: 3.3V from Pi → encoders & IMU
- Common GND: Connect Pi GND to L298N GND
"""

print(correct_hardware)
print(correct_wiring)