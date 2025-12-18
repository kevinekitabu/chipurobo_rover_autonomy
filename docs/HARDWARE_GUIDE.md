# ChipuRobot v0.5 - Hardware Setup Guide

## 📋 Required Components

### Electronics
| Component | Quantity | Description |
|-----------|----------|-------------|
| Raspberry Pi 5 | 1 | Main computer with 4GB+ RAM |
| Raspberry Pi AI Camera | 1 | Primary vision sensor |
| DC Motors | 4 | 6V-12V geared motors |
| H-Bridge Motor Driver | 1 | L298N or similar dual-channel |
| Power Bank | 1 | 5V USB-C for Raspberry Pi |
| 18650 Batteries | 3 | For motor power (with holder) |
| MicroSD Card | 1 | 32GB+ Class 10 for Pi OS |

### Mechanical
| Component | Quantity | Description |
|-----------|----------|-------------|
| Chassis | 1 | Robot frame (3D printed or acrylic) |
| Wheels | 4 | Compatible with DC motors |
| Jumper Wires | 20+ | Male-to-female connections |
| Breadboard | 1 | For connections (optional) |

## 🔌 Wiring Diagram

### GPIO Pin Assignments
```
Raspberry Pi 5 GPIO → Motor Driver
┌─────────────────┬──────────────────────┐
│ GPIO Pin        │ Connection           │
├─────────────────┼──────────────────────┤
│ Pin 17          │ Left Motor Forward   │
│ Pin 27          │ Left Motor Backward  │
│ Pin 22          │ Right Motor Forward  │
│ Pin 23          │ Right Motor Backward │
│ Pin 24          │ Left Enable (PWM)    │
│ Pin 25          │ Right Enable (PWM)   │
│ 5V Power        │ Motor Driver Logic   │
│ GND             │ Common Ground        │
└─────────────────┴──────────────────────┘
```

### Motor Driver (L298N) Connections
```
┌──────────────┬─────────────────┬─────────────────┐
│ L298N Pin    │ Connection      │ Wire Color      │
├──────────────┼─────────────────┼─────────────────┤
│ IN1          │ GPIO 17         │ Orange          │
│ IN2          │ GPIO 27         │ Yellow          │
│ IN3          │ GPIO 22         │ Green           │
│ IN4          │ GPIO 23         │ Blue            │
│ ENA          │ GPIO 24         │ Purple          │
│ ENB          │ GPIO 25         │ Gray            │
│ +12V         │ Battery Pack +  │ Red             │
│ GND          │ Battery Pack -  │ Black           │
│ +5V          │ Pi 5V (if req.) │ Red (thin)      │
└──────────────┴─────────────────┴─────────────────┘
```

## 🔧 Assembly Instructions

### Step 1: Prepare the Chassis
1. **3D Print or Cut Chassis**: Use provided STL files or create custom platform
2. **Mount Motors**: Secure 4 DC motors to chassis corners
3. **Attach Wheels**: Connect wheels to motor shafts
4. **Create Space**: Ensure room for Pi, camera, and motor driver

### Step 2: Install Electronics
1. **Mount Raspberry Pi**: Secure Pi 5 to chassis with standoffs
2. **Mount Camera**: Position AI camera for forward-facing view
3. **Install Motor Driver**: Mount L298N in accessible location
4. **Battery Placement**: Secure 18650 pack away from moving parts

### Step 3: Wiring
1. **Power Connections**:
   ```bash
   # Motor power (18650 pack)
   Red wire → L298N +12V
   Black wire → L298N GND
   
   # Logic power (from Pi or separate 5V)
   Pi 5V → L298N +5V (if needed)
   Pi GND → L298N GND
   ```

2. **Signal Connections** (follow pin table above):
   ```bash
   # Use female-to-male jumper wires
   GPIO 17 → IN1    # Left motor forward
   GPIO 27 → IN2    # Left motor backward  
   GPIO 22 → IN3    # Right motor forward
   GPIO 23 → IN4    # Right motor backward
   GPIO 24 → ENA    # Left motor enable
   GPIO 25 → ENB    # Right motor enable
   ```

3. **Motor Connections**:
   ```bash
   # Left motors (in parallel)
   Motor A+ → L298N OUT1
   Motor A- → L298N OUT2
   
   # Right motors (in parallel)  
   Motor B+ → L298N OUT3
   Motor B- → L298N OUT4
   ```

### Step 4: AI Camera Setup
1. **Connect AI Camera**: Use ribbon cable to Pi camera connector (blue strip away from ethernet)
2. **Install AI Camera Firmware**:
   ```bash
   # Run automated setup (recommended)
   ./scripts/setup_ai_camera.sh
   
   # OR manual installation:
   sudo apt install imx500-all imx500-tools python3-opencv python3-munkres
   ```
3. **Enable Camera**: Run `sudo raspi-config` → Interface Options → Camera → Enable
4. **Reboot**: `sudo reboot` (required for AI Camera)
5. **Test AI Camera**: 
   ```bash
   # Check AI camera detection
   rpicam-hello --list-cameras
   
   # Test AI object detection (should show bounding boxes)
   rpicam-hello -t 10s --post-process-file \
     /usr/share/rpi-camera-assets/imx500_mobilenet_ssd.json \
     --viewfinder-width 640 --viewfinder-height 480
   
   # Test ChipuRobot vision system
   ./run.sh test
   ```

## ⚙️ Software Configuration

### 1. Raspberry Pi OS Setup
```bash
# Update system
sudo apt update && sudo apt upgrade -y

# Install Python dependencies  
sudo apt install python3-pip python3-venv git -y

# Enable camera and GPIO
sudo raspi-config
# → Interface Options → Camera → Enable
# → Interface Options → SSH → Enable (optional)
```

### 2. Install ChipuRobot
```bash
# Clone repository
git clone https://github.com/kevinekitabu/chipurobo_rover_autonomy.git
cd chipurobo_rover_autonomy

# Create virtual environment
python3 -m venv venv
source venv/bin/activate

# Install dependencies
pip install -r requirements.txt
```

### 3. Test Hardware
```bash
# Run system test
python scripts/test_system.py

# Test motors only
python -c "
from chipurobo.hardware.motors import MotorController
motors = MotorController()
motors.forward(1)  # 1 second forward
motors.stop()
"

# Test camera only
python -c "
from chipurobo.vision.camera import VisionProcessor  
vision = VisionProcessor()
frame = vision.capture_frame()
print('Camera working!' if frame is not None else 'Camera failed!')
"
```

## 🔍 Troubleshooting

### Motor Issues
**Motors not moving**:
- Check battery voltage (should be 9-12V)
- Verify GPIO pin connections
- Test with multimeter on motor driver outputs
- Ensure enable pins (ENA, ENB) are HIGH

**Motors moving in wrong direction**:
- Swap motor wires (+ and -)
- Or modify pin assignments in config

**Weak motor response**:
- Check battery charge level
- Verify power supply current capacity
- Reduce motor speed in config

### AI Camera Issues
**AI Camera not detected**:
```bash
# Check camera connection and AI firmware
rpicam-hello --list-cameras
# Should show IMX500 camera

# Verify AI firmware installation
ls /lib/firmware/imx500*
ls /usr/share/imx500-models/

# Reinstall if missing
sudo apt install --reinstall imx500-all

# Check camera enable in raspi-config
sudo raspi-config
```

**AI models not loading**:
```bash
# Check model permissions
sudo chown -R pi:pi /usr/share/imx500-models/

# Verify model files exist
ls -la /usr/share/imx500-models/*.rpk

# Test model loading manually
python3 -c "
from picamera2.devices.imx500 import IMX500
imx500 = IMX500('/usr/share/imx500-models/imx500_network_ssd_mobilenetv2_fpnlite_320x320_pp.rpk')
print('AI model loaded successfully!')
"
```

**Slow AI performance**:
- Ensure 5V 3A power supply (AI Camera needs stable power)
- Check CPU temperature: `vcgencmd measure_temp`
- Consider active cooling for sustained operation
- Verify Pi 5 performance mode: `cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor`

**AI detection not working**:
```bash
# Test ChipuRobot vision system
./run.sh test

# Check vision processor status
python3 -c "
from chipurobo.vision.camera import VisionProcessor
vision = VisionProcessor()
print(vision.get_status())
"
```

**Poor image quality**:
- Clean camera lens
- Adjust lighting conditions
- Modify camera settings in code

### Software Issues
**Import errors**:
```bash
# Reinstall dependencies
pip install --upgrade -r requirements.txt

# Check Python version
python --version  # Should be 3.8+
```

**GPIO permission errors**:
```bash
# Add user to gpio group
sudo usermod -a -G gpio $USER
# Logout and login again
```

## 📏 Physical Specifications

### Recommended Dimensions
- **Length**: 25-30 cm
- **Width**: 20-25 cm  
- **Height**: 15-20 cm
- **Weight**: 1-2 kg (with batteries)

### Performance Specs
- **Speed**: 0.5-2.0 m/s (configurable)
- **Turn Radius**: ~20 cm
- **Battery Life**: 2-4 hours continuous operation
- **Camera Range**: 0.5-5 meters obstacle detection

## 🛡️ Safety Guidelines

### Electrical Safety
- Always disconnect power when wiring
- Use proper gauge wire for motor currents
- Install fuses for battery protection
- Ensure good ventilation for electronics

### Mechanical Safety  
- Secure all components to prevent movement
- Use strain relief on cables
- Ensure emergency stop capability
- Test in open area initially

### Software Safety
- Implement software emergency stops
- Use reasonable speed limits
- Add timeout protections
- Test autonomous modes carefully

---

## 📞 Support

For hardware-specific questions:
1. Check this guide first
2. Review troubleshooting section  
3. Test individual components
4. Open GitHub issue with details

**Remember**: Start simple, test each component individually, then integrate!