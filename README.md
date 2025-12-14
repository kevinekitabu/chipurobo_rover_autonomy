# ChipuRobo - Kenya Science & Engineering Fair 2026 Project

![ChipuRobo](https://img.shields.io/badge/ChipuRobo-KSEF_2026-blue) ![Platform](https://img.shields.io/badge/Platform-Raspberry_Pi-green) ![Competition](https://img.shields.io/badge/Competition-CEMASTEA_Judged-orange) ![Curriculum](https://img.shields.io/badge/Training-40%2B_Hours-red)

**Official 2026 project for Kenya Science and Engineering Fair Scientific Exploration Track.** A comprehensive robotics education platform combining digital fabrication, electronics assembly, and autonomous programming. Participating schools receive **direct entry to KSEF nationals** with judging by CEMASTEA experts.

## � Educational Platform Overview

**ChipuRobo** provides a complete 40+ hour curriculum that takes students from digital design through advanced autonomous programming. Students design, fabricate, assemble, and program their own competition-ready robots.

### 📚 4-Module Curriculum Structure

| Module | Focus Area | Duration | Key Skills |
|--------|------------|----------|------------|
| **1. 3D Printing & CAD** | Digital fabrication and design | 8 hours | Fusion 360, design thinking, manufacturing |
| **2. Laser Cutting & 2D Design** | Precision manufacturing | 6 hours | Vector design, material science, tolerances |
| **3. Electronics & Hardware** | Circuit assembly and integration | 10 hours | Soldering, wiring, system integration |
| **4. Programming & Autonomy** | Software development and AI | 16+ hours | Python, algorithms, computer vision |

### 🎯 Learning Outcomes
- **Design & Fabricate** custom robot chassis using professional CAD tools
- **Assemble & Wire** complex electronic systems safely and professionally  
- **Program Autonomous Behaviors** using industry-standard practices
- **Compete** in FIRST Robotics-style challenges with confidence
- **Collaborate** effectively in technical teams

## 🏗️ Technical Implementation Structure

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

## 🚀 Educational Features & Benefits

### 🎯 Real-World Skills Development
✅ **Professional CAD Design** - Industry-standard Fusion 360 workflows  
✅ **Manufacturing Experience** - 3D printing and laser cutting optimization  
✅ **Electronics Assembly** - Professional soldering and wiring techniques  
✅ **Software Engineering** - Version control, testing, and documentation  
✅ **Project Management** - Portfolio development and team collaboration  

### 🔧 Hardware Platform
- **L298N Motor Driver** - Professional DC motor control with PWM via gpiozero
- **Hall Effect Encoders** - Built-in motor encoders for precise positioning
- **MPU6050/MPU9255 IMU** - 6/9-axis motion sensing for navigation
- **Raspberry Pi Camera** - Computer vision with OpenCV and ArUco detection
- **Raspberry Pi Zero 2 W/Pi 5** - Scalable computing platform
- **gpiozero Library** - Modern, Pythonic GPIO control (no root required)

### 💻 Professional Software Architecture
- **Modular Design** - Clean separation of hardware, sensors, vision, and control
- **Type Hints & Documentation** - Professional Python development practices
- **Comprehensive Testing** - Unit tests and hardware-in-the-loop validation
- **Version Control Integration** - Git workflows and collaborative development
- **Configuration Management** - TOML-based settings for different environments

## 🛠️ GPIO Pin Assignment (Conflict-Free)

```
Left Motor:   PWM=18, IN1=24, IN2=23
Right Motor:  PWM=12, IN1=22, IN2=27
Left Encoder: A=5, B=6
Right Encoder: A=13, B=19
IMU I2C:      SDA=2, SCL=3
Camera:       CSI Interface
```

**Using gpiozero Library** - Modern, Pythonic GPIO interface for Raspberry Pi

## � Kenya Science & Engineering Fair 2026

### 🎯 Competition Benefits
- **Direct Entry to Nationals** - Participating schools automatically qualify for KSEF nationals
- **CEMASTEA Judging** - Evaluation by Kenya's top STEM education experts
- **Scientific Exploration Track** - Focus on research, innovation, and technical excellence
- **National Recognition** - Platform for showcasing student achievements

### 🏫 For Schools & Teachers

#### 📋 Program Structure
- **Beginner Friendly** - No prior robotics experience required
- **Age Range** - Secondary school students (Forms 1-4)
- **Class Size** - Optimized for 20-30 students working in teams
- **Duration** - Full academic year preparation with weekly training sessions

#### 📚 Teacher Support
- **Virtual Training Sessions** - Weekly online sessions leading up to nationals
- **Technical Support** - Ongoing assistance throughout the program
- **Curriculum Materials** - Complete lesson plans and assessment guides
- **Competition Preparation** - Structured pathway to KSEF nationals

### 💰 Program Packages (2026 Pricing)

#### **Entry Level Package - KES 85,000**
**Perfect for schools starting their robotics journey**
```
✅ Complete hardware kit for one robot team
✅ All electronic components and sensors
✅ Raspberry Pi with pre-configured software
✅ Weekly virtual training sessions for teachers
✅ Technical support throughout the program
✅ KSEF competition registration and guidance
✅ Direct qualification pathway to nationals
```

#### **Center of Excellence Package - KES 500,000** 
**One-time investment for sustained robotics programs**
```
✅ Everything in Entry Level Package
✅ 2 x Bambu Lab 3D Printers for manufacturing
✅ Complete makerspace setup with tools
✅ Hardware kits for multiple teams
✅ Multi-year project sustainability
✅ Priority technical support and training
✅ Annual project updates and new challenges
```

### 🔄 Program Continuity
- **Annual Projects** - New challenges each year for sustained engagement
- **2026 Focus** - ChipuRobo autonomous navigation and AI
- **Future Years** - Evolving projects building on established hardware base
- **Skill Progression** - Students advance from basic to advanced robotics concepts

## ⚡ Quick Start (Technical Implementation)

### 1. Clone and Setup
```bash
git clone https://github.com/kevinekitabu/chipurobo_rover_autonomy.git
cd chipurobo_rover_autonomy
pip install -r requirements.txt
```

### 2. Deploy to Raspberry Pi
```bash
# Deploy complete system to Pi
python scripts/deploy_to_pi.py raspberrypi.local

# Or manually on Pi:
python scripts/run_server.py --config config/production.toml
```

### 3. Access Mission Control
```bash
# Open web interface at:
http://raspberrypi.local:5001
# or http://localhost:5001
```

## � Educational Outcomes & Assessment

### 🎯 Student Learning Objectives
1. **Design Thinking** - Systematic approach to engineering problem-solving
2. **Digital Fabrication** - CAD design, 3D printing, and laser cutting proficiency
3. **Systems Integration** - Electronics assembly and troubleshooting skills
4. **Software Engineering** - Professional Python development practices
5. **Project Management** - Portfolio development and team collaboration
6. **Competition Readiness** - FIRST Robotics Competition preparation

### 📊 Assessment Methods
- **Portfolio-Based Assessment** (60%) - Comprehensive project documentation
- **Practical Demonstrations** (25%) - Live technical skill assessments  
- **Competition Performance** (15%) - Autonomous navigation challenges
- **Peer Collaboration** - Team-based project evaluation

### 🌍 Kenya STEM Education Impact
- **CEMASTEA Alignment** - Directly supports Kenya's STEM education goals
- **University Preparation** - Strong foundation for engineering programs at Kenyan universities
- **Industry Readiness** - Skills aligned with Kenya's growing technology sector
- **Innovation Culture** - Fostering the next generation of Kenyan innovators and entrepreneurs

### 🔗 Beyond Competition
- **University Pathways** - Enhanced applications for engineering programs
- **Continued Learning** - Foundation for advanced robotics and AI studies
- **Entrepreneurship** - Skills for Kenya's startup and innovation ecosystem
- **Regional Leadership** - Positioning Kenya as East Africa's STEM education hub

### 📞 Get Started for KSEF 2026

**Ready to participate in Kenya's premier science competition?**

- **School Registration** - Contact us to register for KSEF 2026
- **Teacher Training** - Join weekly virtual sessions starting January 2026
- **Technical Support** - Get help throughout your robotics journey
- **Competition Preparation** - Structured pathway to nationals success

This platform prepares Kenyan students for excellence in the Kenya Science and Engineering Fair while building lasting STEM capabilities for the nation's future.
