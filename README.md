# ChipuRobo - Autonomous Robot Control System

![ChipuRobo](https://img.shields.io/badge/ChipuRobo-v1.0.0-blue) ![Platform](https://img.shields.io/badge/Platform-Raspberry_Pi-green) ![License](https://img.shields.io/badge/License-MIT-yellow)

A professional robotics framework for autonomous navigation and mission control, designed for FIRST Robotics Competition style challenges.

## 🏗️ Professional Project Structure

```
chipurobo_rover_autonomy/
├── chipurobo/                 # Main Python package
│   ├── hardware/             # Hardware control modules
│   │   ├── gpio_manager.py   # GPIO pin management
│   │   ├── motors.py         # L298N motor driver
│   │   ├── encoders.py       # Motor encoder interface
│   │   └── robot.py          # Main robot class
│   ├── sensors/              # Sensor interfaces
│   │   └── imu.py           # MPU9255 IMU
│   ├── vision/              # Computer vision
│   │   └── camera.py        # Pi Camera + ArUco
│   ├── control/             # Navigation & control
│   ├── mission/             # Mission planning
│   └── utils/               # Utilities
│       ├── config_manager.py # Configuration management
│       └── logger.py        # Professional logging
├── server/                  # Mission control server
│   └── app.py              # Flask backend API
├── web/                    # Web interface
│   ├── templates/          # HTML templates
│   │   └── mission_control.html # Web-based mission control
│   └── static/            # CSS, JS, images
├── tools/                 # Development utilities
│   └── path_editor.py    # GUI path planning tool
├── scripts/               # Entry point scripts
│   ├── run_robot.py      # Robot control script
│   ├── run_server.py     # Server launcher
│   └── deploy_to_pi.py   # Raspberry Pi deployment
├── config/               # Configuration files
│   ├── production.toml   # Production settings
│   └── development.toml  # Development settings
├── deploy/               # Deployment assets
│   └── pathplanner/     # Path planning data
│       ├── navgrid.json
│       ├── autos/
│       └── paths/
├── tests/               # Unit tests
├── docs/               # Documentation
├── requirements.txt    # Python dependencies
└── README.md          # This file
```

## 🚀 Key Improvements

### Fixed Issues
✅ **GPIO Pin Conflicts Resolved** - Centralized pin management prevents conflicts  
✅ **Daemon Thread Issue Fixed** - Proper thread lifecycle management  
✅ **Modular Architecture** - Clean separation of concerns  
✅ **Professional Logging** - Structured logging with rotation  
✅ **Configuration Management** - TOML-based settings  

### Hardware Support
- **L298N Motor Driver** - Professional DC motor control with PWM
- **Hall Effect Encoders** - Built-in motor encoders for precise positioning
- **MPU9255 9-Axis IMU** - Gyroscope, accelerometer, magnetometer
- **Raspberry Pi AI Camera** - Computer vision with ArUco marker detection

### Software Architecture
- **Type Hints** - Full type annotation
- **Error Handling** - Comprehensive exception management
- **Documentation** - Docstrings and inline comments
- **Testing Structure** - Ready for unit tests
- **Entry Points** - Professional CLI scripts

## 🛠️ GPIO Pin Assignment (Conflict-Free)

```
Left Motor:   PWM=18, IN1=24, IN2=23
Right Motor:  PWM=12, IN1=22, IN2=27
Left Encoder: A=5, B=6
Right Encoder: A=13, B=19
IMU I2C:      SDA=2, SCL=3
Camera:       CSI Interface
```

## ⚡ Quick Start

### 1. Installation
```bash
pip install flask flask-cors toml
```

### 2. Run System
```bash
# Start robot (test mode)
python scripts/run_robot.py --test

# Start mission control server  
python scripts/run_server.py

# Open web interface at http://localhost:5001
```

## 🎯 Professional Features

1. **Modular Design** - Each component is independently testable
2. **Configuration Management** - Environment-specific settings
3. **Centralized Logging** - Professional logging with rotation
4. **Mission Control** - Web-based planning and execution
5. **Real-Time Telemetry** - Live robot status monitoring
6. **Sensor Fusion** - Combined positioning from multiple sources

This restructured project now follows Python best practices and professional software development standards.
