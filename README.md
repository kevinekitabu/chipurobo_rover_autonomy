# ChipuRobot v0.5 - Computer Vision Autonomous Rover

[![Python 3.8+](https://img.shields.io/badge/python-3.8+-blue.svg)](https://www.python.org/downloads/)
[![OpenCV](https://img.shields.io/badge/OpenCV-4.8+-green.svg)](https://opencv.org/)
[![Raspberry Pi](https://img.shields.io/badge/Raspberry%20Pi-5-red.svg)](https://www.raspberrypi.org/)

**Built for Kenya Science & Engineering Fair (KSEF) 2025**

An educational autonomous robot that demonstrates computer vision-driven decision making. ChipuRobot v0.5 uses its camera as the primary sensor to perceive the environment and make intelligent movement decisions in real-time.

## 🎯 Project Overview

**Core Concept**: Computer Vision → Intelligence → Movement

ChipuRobot v0.5 showcases how artificial intelligence can control robot behavior through visual perception alone, making it perfect for educational demonstrations and science fair presentations.

### Key Features
- 🧠 **AI-Powered Vision**: Uses Raspberry Pi AI Camera (IMX500) with neural network processing
- 🔍 **Smart Obstacle Avoidance**: Combines edge detection with AI object recognition
- 🎯 **Intelligent Following**: AI-powered person detection and tracking (80+ object classes)
- ⚡ **Real-Time Performance**: 30 FPS AI inference with low-latency decisions
- 🎮 **Manual Override**: Instant switch between autonomous and manual control
- 🔧 **Simulation Mode**: Full functionality without hardware for development

## 🚀 Quick Start

### Prerequisites
- Python 3.8+
- Raspberry Pi 5 (for hardware deployment)
- Raspberry Pi AI Camera
- Motor driver and DC motors

### Installation
```bash
git clone https://github.com/kevinekitabu/chipurobo_rover_autonomy.git
cd chipurobo_rover_autonomy
pip install -r requirements.txt
```

### Run Examples
```bash
# Easy way (recommended)
./run.sh test          # System test
./run.sh demo          # Quick demo  
./run.sh interactive   # Interactive control
./run.sh ksef          # KSEF presentation

# Direct Python (set PYTHONPATH=. first)
PYTHONPATH=. python3 scripts/test_system.py
PYTHONPATH=. python3 examples/quick_demo.py
PYTHONPATH=. python3 scripts/main.py
PYTHONPATH=. python3 scripts/ksef_demo.py
```

## 🧠 Autonomous Modes

### 1. Vision-Based Obstacle Avoidance
```
Camera Input → Edge Detection → Decision Logic → Motor Command
```
- **Forward**: Path is clear
- **Turn Left**: Obstacle on right or ahead with left path clear
- **Turn Right**: Obstacle on left or ahead with right path clear  
- **Stop**: Obstacles everywhere or processing error

### 2. Vision-Based Object Following  
```
Camera Input → Color/Object Detection → Tracking Logic → Motor Command
```
- **Turn Left**: Target object on left side of camera
- **Turn Right**: Target object on right side of camera
- **Forward**: Target centered and far away
- **Stop**: Target centered and close (mission complete)

## 🏗️ Project Structure

```
chipurobo_rover_autonomy/
├── README.md                    # This file
├── requirements.txt             # Python dependencies
├── .gitignore                  # Git ignore rules
│
├── chipurobo/                  # Main robot package
│   ├── __init__.py
│   ├── hardware/               # Hardware control
│   │   ├── robot.py           # Main robot class
│   │   └── motors.py          # Motor controller
│   ├── vision/                # Computer vision
│   │   └── camera.py          # Vision processor
│   └── utils/                 # Utilities
│       ├── logger.py          # Logging system
│       └── config_manager.py  # Configuration
│
├── config/                    # Configuration files
│   └── development.toml       # Robot settings
│
├── scripts/                   # Executable scripts
│   ├── main.py               # Interactive control
│   ├── ksef_demo.py          # KSEF presentation
│   └── test_system.py        # System tests
│
├── examples/                  # Example code
│   └── quick_demo.py         # Simple demo
│
└── docs/                     # Documentation
    ├── HARDWARE_GUIDE.md     # Hardware setup
    ├── API_REFERENCE.md      # Code documentation
    └── TROUBLESHOOTING.md    # Common issues
```

## 🚀 Quick Start

### 1. Installation
```bash
# On Raspberry Pi 5
git clone <repository>
cd chipurobo_rover_autonomy
pip install -r requirements.txt
```

### 2. Basic Operation
```bash
# Interactive mode
python3 main_v05.py

# Automated KSEF demo
python3 ksef_demo.py
```

### 3. Controls
```
Controls in main_v05.py:
- Enter: Start/Stop autonomous mode
- 'o': Switch to obstacle avoidance  
- 'f': Switch to object following
- 'w','a','d','x': Manual forward/left/right/stop
- 'q': Quit
```

## 🎯 Educational Value

### For KSEF Judges & Audience
1. **Clear Cause-Effect**: Camera image → AI decision → Robot movement
2. **No Black Box**: Simple, explainable computer vision algorithms
3. **Real-Time Demo**: Live video processing and decision making
4. **Scalable Concept**: Foundation for advanced robotics

### Technical Learning Points
- **Computer Vision**: Edge detection, color filtering, object tracking
- **Decision Logic**: If-then rules based on visual input
- **Control Systems**: Feedback loop from vision to motors
- **Real-Time Processing**: 5Hz decision cycle for responsive behavior

## 🔧 Configuration

Key settings in `config/development.toml`:
```toml
[robot]
motor_speed = 0.8          # Motor speed (0.0 to 1.0)
vision_mode = "obstacle_avoidance"  # or "object_following"

[vision]
decision_interval = 0.2    # Seconds between vision decisions
obstacle_threshold = 0.3   # Edge density threshold for obstacles
target_zone_width = 100    # Pixel width for "centered" target
```

## 🎬 Demo Scenarios

### Scenario 1: Obstacle Course
1. Set up boxes/barriers in random pattern
2. Robot navigates using camera vision only
3. Demonstrates spatial reasoning through computer vision

### Scenario 2: Follow the Leader
1. Person holds bright colored object
2. Robot follows the object around
3. Demonstrates target tracking and pursuit behavior

## 🚧 Simulation Mode

The robot runs in simulation mode for development:
- **No Hardware Required**: Test logic on any computer
- **Simulated Vision**: Test patterns for algorithm development  
- **Full Functionality**: Complete code path testing
- **Easy Debugging**: Perfect for code development

## � Hardware Setup

### Required Components
- **Raspberry Pi 5** - Main computer
- **Raspberry Pi AI Camera** - Vision sensor
- **4× DC Motors** - Movement (wired as 2 channels)
- **H-Bridge Motor Driver** - Motor control
- **Power Bank** - Pi power supply
- **3×18650 Battery Pack** - Motor power supply

### Wiring
```
Raspberry Pi GPIO → Motor Driver
Pin 17 → Left Motor Forward
Pin 27 → Left Motor Backward  
Pin 22 → Right Motor Forward
Pin 23 → Right Motor Backward
Pin 24 → Left Enable
Pin 25 → Right Enable
```

See [docs/HARDWARE_GUIDE.md](docs/HARDWARE_GUIDE.md) for complete setup instructions.

## 🧠 Vision Processing

### Obstacle Detection Algorithm
1. **Image Capture**: Get frame from Pi Camera at 30fps
2. **Preprocessing**: Convert to grayscale, apply Gaussian blur
3. **Edge Detection**: Use Canny edge detection
4. **Region Analysis**: Divide into left/center/right sections
5. **Density Calculation**: Count edge pixels per section
6. **Decision Logic**: Choose movement based on obstacle density

### Object Tracking Algorithm  
1. **Color Space Conversion**: RGB to HSV for better color detection
2. **Color Filtering**: Create mask for target color range
3. **Contour Detection**: Find object boundaries
4. **Centroid Calculation**: Determine object center
5. **Position Analysis**: Compare to frame center for steering decisions

## ⚙️ Configuration

Edit `config/development.toml` to customize robot behavior:

```toml
[robot]
motor_speed = 0.8              # Speed (0.0 to 1.0)
vision_mode = "obstacle_avoidance"  # or "object_following"

[vision]
decision_interval = 0.2        # Seconds between decisions
obstacle_threshold = 0.3       # Edge density threshold for obstacles
target_zone_width = 100        # Pixel width for "centered" target

[hardware]
left_forward_pin = 17
left_backward_pin = 27
right_forward_pin = 22
right_backward_pin = 23
```

## 🎬 Demo Scenarios

### Scenario 1: Obstacle Course
1. Set up boxes/barriers in random pattern
2. Robot navigates using camera vision only
3. Demonstrates spatial reasoning through computer vision

### Scenario 2: Follow the Leader
1. Person holds bright colored object
2. Robot follows the object around
3. Demonstrates target tracking and pursuit behavior

## 🚧 Simulation Mode

The robot runs in simulation mode for development:
- **No Hardware Required**: Test logic on any computer
- **Simulated Vision**: Test patterns for algorithm development  
- **Full Functionality**: Complete code path testing
- **Easy Debugging**: Perfect for code development

## 🚨 Troubleshooting

### Common Issues

**Camera not detected**:
```bash
# Check camera connection
libcamera-hello --list-cameras

# Verify PiCamera2 installation
pip install picamera2
```

**Motor not responding**:
- Check wiring connections
- Verify GPIO pin assignments
- Test with manual control first

**Import errors**:
```bash
# Install missing dependencies
pip install -r requirements.txt

# Run system test
python scripts/test_system.py
```

See [docs/TROUBLESHOOTING.md](docs/TROUBLESHOOTING.md) for complete troubleshooting guide.

## 📚 Documentation

- [Hardware Setup Guide](docs/HARDWARE_GUIDE.md) - Complete wiring and assembly
- [API Reference](docs/API_REFERENCE.md) - Code documentation and examples  
- [Troubleshooting](docs/TROUBLESHOOTING.md) - Common issues and solutions

## 📚 Code Examples

### Basic Robot Usage
```python
from chipurobo.hardware.robot import ChipuRobot

# Initialize robot
robot = ChipuRobot()

# Start autonomous obstacle avoidance
robot.set_vision_mode('obstacle_avoidance')
robot.start_autonomous_mode()

# Let it run for 30 seconds
time.sleep(30)

# Clean shutdown
robot.cleanup()
```

### Manual Control
```python
# Manual driving
robot.manual_control('forward', duration=2.0)
robot.manual_control('turn_left')
robot.manual_control('stop')
```

## 🎓 Educational Impact

**Perfect for Science Fairs Because:**
- Judges can see AI making real-time decisions
- Clear connection between input (camera) and output (movement)
- Demonstrates practical AI application
- Inspires follow-up questions about computer vision
- Shows path to more advanced robotics

## 📞 Support & Development

Built by **Kevin Irungu** for educational robotics and KSEF 2025.

**Project Goals:**
- Inspire students to explore computer vision and robotics
- Demonstrate practical artificial intelligence applications  
- Provide foundation for advanced autonomous systems
- Create engaging STEM education platform

---

**ChipuRobot v0.5**: *Where Computer Vision Meets Motion* 🤖👁️🚀