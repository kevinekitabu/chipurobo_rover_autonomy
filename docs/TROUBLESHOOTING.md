# ChipuRobot v0.5 - Troubleshooting Guide

## 🔍 Common Issues & Solutions

### 🐍 Python Environment Issues

#### ImportError: Module not found
```bash
# Error
ImportError: No module named 'cv2'
ImportError: No module named 'picamera2'  
ImportError: No module named 'gpiozero'

# Solution
pip install --upgrade -r requirements.txt

# If still failing, try:
pip install opencv-python picamera2 gpiozero numpy toml
```

#### Python version incompatibility
```bash
# Check Python version
python --version

# Should be Python 3.8 or higher
# If not, install newer Python:
sudo apt update
sudo apt install python3.9 python3.9-pip
python3.9 -m pip install -r requirements.txt
```

#### Virtual environment issues
```bash
# Create clean virtual environment
python3 -m venv chipurobo_env
source chipurobo_env/bin/activate
pip install --upgrade pip
pip install -r requirements.txt
```

---

### 📷 Camera Issues

#### Camera not detected
```bash
# Error
Camera not available - using simulation mode

# Check camera connection
libcamera-hello --list-cameras

# If no cameras detected:
# 1. Check ribbon cable connection
# 2. Enable camera in raspi-config
sudo raspi-config
# → Interface Options → Camera → Enable → Reboot
```

#### Camera permission denied
```bash
# Error  
Permission denied: '/dev/video0'

# Solution - add user to video group
sudo usermod -a -G video $USER
# Log out and log back in

# Or run with sudo (not recommended for development)
sudo python scripts/main.py
```

#### Poor image quality
```bash
# Symptoms
# - Blurry images
# - Dark images  
# - Color issues

# Solutions
# 1. Clean camera lens with soft cloth
# 2. Adjust lighting - need good illumination
# 3. Check camera focus (some cameras have adjustable focus)
# 4. Modify camera settings in code:

from chipurobo.vision.camera import VisionProcessor
vision = VisionProcessor()
# Adjust exposure, gain, etc. in camera initialization
```

#### Camera initialization fails
```bash
# Error
RuntimeError: Failed to initialize camera

# Solutions
# 1. Check if another process is using camera
sudo lsof /dev/video*
# Kill other processes if needed

# 2. Restart camera service
sudo systemctl restart camera

# 3. Reboot Pi
sudo reboot
```

---

### 🚗 Motor Control Issues

#### Motors not moving
```bash
# Symptoms
# - No motor response to commands
# - Motors make noise but don't turn

# Check 1: Power supply
# Measure battery voltage (should be 9-12V)
# Check connections to motor driver

# Check 2: GPIO connections  
# Verify wiring matches pin assignments:
# GPIO 17 → IN1, GPIO 27 → IN2, etc.

# Check 3: Enable pins
# GPIO 24 & 25 should be HIGH for motor driver enable

# Test individual components:
python -c "
from chipurobo.hardware.motors import MotorController
motors = MotorController()
motors.stop()
print('Testing motor controller...')
motors.forward(1)
"
```

#### Motors moving in wrong direction
```bash
# Symptoms
# - Robot turns opposite direction
# - Forward command moves backward

# Solution 1: Swap motor wires
# Switch + and - wires on problematic motors

# Solution 2: Modify code
# Edit chipurobo/hardware/motors.py
# In turn_left() method, swap which motors go forward/backward
```

#### Weak motor response
```bash
# Symptoms  
# - Motors barely move
# - Inconsistent movement

# Check 1: Battery charge
# Measure voltage under load
# Replace batteries if voltage drops below 9V

# Check 2: Motor driver heat
# L298N gets hot - ensure adequate cooling
# Add heatsink if necessary

# Check 3: Current capacity
# Ensure battery pack can supply required current
# Typical 4-motor setup needs 2-4A peak current

# Check 4: PWM settings
# Increase motor speed in config:
motor_speed = 1.0  # Maximum speed
```

---

### 🧠 Vision Processing Issues  

#### Slow vision processing
```bash
# Symptoms
# - Delayed responses
# - Jerky movement
# - CPU usage high

# Solution 1: Reduce image resolution
# Edit chipurobo/vision/camera.py
# Change frame_width, frame_height to smaller values

# Solution 2: Increase decision interval
# Edit config/development.toml
decision_interval = 0.5  # Slower processing

# Solution 3: Optimize OpenCV
# Install optimized OpenCV
pip uninstall opencv-python
pip install opencv-python-headless

# Solution 4: Reduce processing complexity
# Simplify edge detection parameters
# Skip some processing steps in simulation mode
```

#### Vision decisions not working
```bash
# Symptoms
# - Robot always goes forward
# - No obstacle avoidance
# - No object following

# Debug vision decisions:
python -c "
from chipurobo.vision.camera import VisionProcessor
vision = VisionProcessor()
decision = vision.process_frame_for_autonomy()
print(f'Action: {decision.action}')  
print(f'Reason: {decision.reason}')
print(f'Confidence: {decision.confidence}')
"

# Check thresholds in config:
obstacle_threshold = 0.2  # Lower = more sensitive
target_zone_width = 150   # Wider = easier to center
```

#### Object following not working
```bash
# Symptoms
# - Robot doesn't follow colored objects
# - No target detection

# Check color ranges
# Edit detect_person_or_object() in camera.py
# Adjust HSV color ranges for your target object

# Test different colored objects:
# - Bright red works best initially
# - Avoid colors similar to background
# - Ensure good lighting

# Debug target detection:
python -c "
from chipurobo.vision.camera import VisionProcessor
import cv2
vision = VisionProcessor()
vision.set_mode('object_following')

frame = vision.capture_frame()
decision = vision.detect_person_or_object(frame)
print(f'Target detected: {decision.target_detected}')
"
```

---

### ⚙️ Configuration Issues

#### Config file not found
```bash
# Error
Config file not found: /path/to/config/development.toml

# Solution 1: Check file exists
ls config/
# Should show development.toml

# Solution 2: Create default config
mkdir -p config
cat > config/development.toml << EOF
[robot]
motor_speed = 0.8
vision_mode = "obstacle_avoidance"

[vision]  
decision_interval = 0.2
obstacle_threshold = 0.3
target_zone_width = 100
EOF
```

#### Invalid config values
```bash
# Error
ValueError: Speed must be between 0.0 and 1.0

# Check config values are in valid ranges:
motor_speed = 0.8        # 0.0 to 1.0
decision_interval = 0.2  # > 0.0
obstacle_threshold = 0.3 # 0.0 to 1.0
```

---

### 🔌 GPIO & Hardware Issues

#### GPIO permission denied
```bash
# Error
RuntimeError: No access to /dev/mem

# Solution
sudo usermod -a -G gpio $USER
# Log out and log back in

# Or install proper permissions
sudo apt install rpi.gpio-common
```

#### GPIO pins already in use
```bash
# Error  
RuntimeError: GPIO pin already in use

# Check what's using GPIO
sudo lsof /dev/gpiomem

# Kill other processes or change pin assignments in config
```

#### Hardware simulation mode
```bash
# Message
Running in simulation mode - no GPIO hardware

# This is normal on non-Raspberry Pi systems
# For development, simulation mode works fine
# On Raspberry Pi, check:
# 1. GPIO hardware enabled
# 2. gpiozero installed properly
# 3. Running on actual Pi (not in container)
```

---

### 🚀 Performance Issues

#### System running slowly
```bash
# Check system resources
htop
# Look for high CPU/memory usage

# Check temperature
vcgencmd measure_temp
# Should be below 60°C

# Free up memory
sudo apt autoremove
sudo apt autoclean

# Increase swap if needed  
sudo dphys-swapfile swapoff
sudo sed -i 's/CONF_SWAPSIZE=100/CONF_SWAPSIZE=512/' /etc/dphys-swapfile
sudo dphys-swapfile setup
sudo dphys-swapfile swapon
```

#### Autonomous mode freezing
```bash
# Symptoms
# - Robot stops responding
# - Must restart program

# Check for infinite loops in vision processing
# Enable debug logging:
# Edit chipurobo/utils/logger.py
# Set level to DEBUG

# Add timeout protection:
# Edit autonomous_control_loop in robot.py
# Add watchdog timer or exception handling
```

---

### 🐛 Debugging Techniques

#### Enable verbose logging
```python
# At start of your script:
import logging
logging.basicConfig(level=logging.DEBUG)

# Or modify config/development.toml:
[logging]
level = "DEBUG"
```

#### Test components individually
```python
# Test motors only
from chipurobo.hardware.motors import MotorController
motors = MotorController()
motors.forward(1)

# Test vision only  
from chipurobo.vision.camera import VisionProcessor
vision = VisionProcessor()
frame = vision.capture_frame()

# Test config loading
from chipurobo.utils.config_manager import load_config
config = load_config()
print(config)
```

#### Use system test script
```bash
# Run comprehensive system test
python scripts/test_system.py

# This tests all components and reports issues
```

#### Monitor in real-time
```python
# Add print statements to see what's happening
from chipurobo.hardware.robot import ChipuRobot

robot = ChipuRobot()
robot.start_autonomous_mode()

# Watch status in real-time
import time
while True:
    status = robot.get_robot_status()
    print(f"Mode: {status['vision_status']['mode']}, "
          f"Moving: {status['motor_status']['is_moving']}")
    time.sleep(1)
```

---

### 📞 Getting Help

#### Before asking for help:

1. **Run system test**:
   ```bash
   python scripts/test_system.py
   ```

2. **Check basic connectivity**:
   ```bash
   # Camera
   libcamera-hello --list-cameras
   
   # GPIO  
   pinout  # Shows GPIO pin layout
   
   # Python environment
   pip list | grep -E "(opencv|picamera|gpiozero)"
   ```

3. **Enable debug logging** and capture output

4. **Note your setup**:
   - Raspberry Pi model
   - Camera model  
   - Motor driver type
   - Python version
   - OS version (`cat /etc/os-release`)

#### Where to get help:

1. **Check this troubleshooting guide first**
2. **Review API documentation** (`docs/API_REFERENCE.md`)
3. **Run example scripts** (`examples/quick_demo.py`)
4. **Open GitHub issue** with full debug information

#### GitHub issue template:
```markdown
## Problem Description
Brief description of the issue

## System Information  
- Raspberry Pi model: 
- Camera: 
- Python version: 
- OS: 

## Steps to Reproduce
1. 
2. 
3. 

## Expected vs Actual Behavior
Expected: 
Actual:

## Debug Output
```
paste relevant log output or test results
```

## What I've Tried
- [ ] Ran system test
- [ ] Checked hardware connections  
- [ ] Reviewed troubleshooting guide
- [ ] Tested individual components
```

---

Remember: **Start simple, test incrementally, and don't hesitate to ask for help!**