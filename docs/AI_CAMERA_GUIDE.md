# ChipuRobot v0.5 - AI Camera (IMX500) Guide

## 🤖 Overview

The Raspberry Pi AI Camera with Sony IMX500 sensor provides ChipuRobot with advanced computer vision capabilities through on-camera neural network processing. This guide covers setup, configuration, and optimization for autonomous robotics applications.

## 🏗️ Technical Specifications

### Hardware
- **Sensor**: Sony IMX500 (12.3MP)
- **AI Accelerator**: Integrated neural processing unit
- **Interface**: MIPI CSI-2 (2-lane)
- **Power**: 5V via Raspberry Pi camera connector
- **Dimensions**: 25mm x 24mm x 11.5mm
- **Weight**: 6g

### Performance
- **Inference Speed**: Real-time at 30 FPS
- **Latency**: <10ms for object detection
- **Power Consumption**: ~1.5W during inference
- **Supported Models**: MobileNet SSD, YOLO, PoseNet, Custom models

## 📦 Installation and Setup

### Prerequisites
- Raspberry Pi 4B or Raspberry Pi 5 (recommended)
- Raspberry Pi OS Bullseye or later
- Stable 5V 3A power supply
- 32GB+ microSD card (Class 10)

### Automated Installation
```bash
# Clone ChipuRobot repository
git clone https://github.com/kevinekitabu/chipurobo_rover_autonomy.git
cd chipurobo_rover_autonomy

# Run automated AI Camera setup
./scripts/setup_ai_camera.sh

# Reboot system
sudo reboot
```

### Manual Installation
```bash
# Update system packages
sudo apt update && sudo apt full-upgrade -y

# Install AI Camera firmware and tools
sudo apt install imx500-all imx500-tools

# Install Python dependencies
sudo apt install python3-opencv python3-munkres python3-cv-bridge

# Enable camera interface
sudo raspi-config
# Navigate to: Interface Options → Camera → Enable

# Reboot to load firmware
sudo reboot
```

## 🔧 Configuration

### ChipuRobot Configuration
Edit `config/raspberry_pi5.toml`:

```toml
[camera]
type = "imx500"
resolution = [640, 480]
framerate = 30
format = "RGB888"

# AI processing settings
ai_enabled = true
object_detection_model = "/usr/share/imx500-models/imx500_network_ssd_mobilenetv2_fpnlite_320x320_pp.rpk"
pose_estimation_model = "/usr/share/imx500-models/imx500_network_higherhrnet_coco_pose_640x480_pp.rpk"
detection_threshold = 0.5
inference_roi_auto = true

# Vision processing parameters
obstacle_threshold = 0.3
target_zone_width = 100
min_target_size = 1000
processing_threads = 2
```

### Performance Optimization
```bash
# Set CPU performance mode
echo performance | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor

# Increase GPU memory split for camera processing
sudo raspi-config
# Advanced Options → Memory Split → 128

# Optional: Enable overclocking (monitor temperatures!)
# Add to /boot/config.txt:
# over_voltage=2
# arm_freq=2000
```

## 🧠 AI Models

### Pre-installed Models
The IMX500 comes with several pre-trained neural network models:

#### Object Detection (MobileNet SSD)
- **File**: `imx500_network_ssd_mobilenetv2_fpnlite_320x320_pp.rpk`
- **Classes**: 80 COCO dataset objects (person, car, bicycle, etc.)
- **Input Size**: 320x320 pixels
- **Performance**: 30 FPS real-time detection
- **Use Case**: General object detection and obstacle avoidance

#### Pose Estimation (HigherHRNet)
- **File**: `imx500_network_higherhrnet_coco_pose_640x480_pp.rpk`
- **Function**: Human pose detection with 17 keypoints
- **Input Size**: 640x480 pixels
- **Performance**: 15-30 FPS depending on scene complexity
- **Use Case**: Human following and gesture recognition

### Custom Model Deployment
```bash
# Install model development toolkit
pip install edge-mdt[pt]  # For PyTorch models
# OR
pip install edge-mdt[tf]  # For TensorFlow models

# Convert and package custom model (advanced users)
# See: https://developer.aitrios.sony-semicon.com/en/raspberrypi-ai-camera
```

## 💻 Programming Interface

### Basic Usage
```python
from chipurobo.vision.camera import VisionProcessor

# Initialize AI-enabled vision processor
vision = VisionProcessor()

# Check AI status
status = vision.get_status()
print(f"AI Enabled: {status['ai_enabled']}")
print(f"IMX500 Ready: {status['imx500_ready']}")

# Process frame for autonomous decision
decision = vision.process_frame_for_autonomy()
print(f"Action: {decision.action}")
print(f"Reason: {decision.reason}")
print(f"Confidence: {decision.confidence}")
```

### Advanced AI Features
```python
# Access raw AI inference data
if vision.ai_enabled:
    request = vision.camera.capture_request()
    
    # Get AI outputs
    outputs = vision.imx500.get_outputs(request.get_metadata())
    boxes, scores, classes = outputs[0][0], outputs[1][0], outputs[2][0]
    
    # Process detections
    for box, score, cls in zip(boxes, scores, classes):
        if score > 0.5:  # Confidence threshold
            print(f"Detected: {vision._get_class_name(int(cls))} "
                  f"(confidence: {score:.2f})")
    
    request.release()
```

## 📊 Monitoring and Debugging

### Performance Monitoring
```python
# Get AI performance metrics
metadata = request.get_metadata()
kpi_info = vision.imx500.get_kpi_info(metadata)
print(f"Inference time: {kpi_info}")

# Monitor system resources
import psutil
print(f"CPU usage: {psutil.cpu_percent()}%")
print(f"Memory usage: {psutil.virtual_memory().percent}%")
print(f"Temperature: {psutil.sensors_temperatures()}")
```

### Debug Mode
```bash
# Run with debug output
CHIPUROBO_DEBUG=1 ./run.sh demo

# Test individual components
./run.sh test --component vision
./run.sh test --component ai
```

## 🔍 Troubleshooting

### Common Issues

#### 1. Camera Not Detected
```bash
# Check camera detection
rpicam-hello --list-cameras

# Should output something like:
# 0 : imx500 [4056x3040 10-bit RGGB] (/base/soc/i2c0mux/i2c@1/imx500@1a)
#     Modes: 'SRGGB10_CSI2P' : 1332x990 [30.00 fps - (1362, 1025)/1332x990 crop]

# If not detected, check:
# 1. Cable connection (blue strip away from ethernet)
# 2. Camera enabled: sudo raspi-config → Interface → Camera
# 3. Reboot after enabling
```

#### 2. AI Models Not Loading
```bash
# Check firmware files
ls -la /lib/firmware/imx500*
# Should show: imx500_firmware.fpk, imx500_loader.fpk

# Check model files
ls -la /usr/share/imx500-models/
# Should show various .rpk files

# Reinstall if missing
sudo apt install --reinstall imx500-all

# Test model loading
python3 -c "
from picamera2.devices.imx500 import IMX500
try:
    imx500 = IMX500('/usr/share/imx500-models/imx500_network_ssd_mobilenetv2_fpnlite_320x320_pp.rpk')
    print('✅ AI model loaded successfully!')
except Exception as e:
    print(f'❌ Model loading failed: {e}')
"
```

#### 3. Poor AI Performance
- **Power Supply**: Ensure stable 5V 3A supply
- **Thermal**: Monitor temperature with `vcgencmd measure_temp`
- **CPU Governor**: Set to performance mode
- **Memory**: Increase GPU memory split to 128MB or higher

#### 4. Inaccurate Detections
```python
# Adjust detection threshold
vision.detection_threshold = 0.7  # More strict
# OR
vision.detection_threshold = 0.3  # More permissive

# Set specific region of interest
# Focus on specific area for better performance
vision.imx500.set_inference_roi_abs((100, 100, 400, 300))  # x, y, width, height
```

### Hardware Diagnostics
```bash
# Check camera hardware
vcgencmd get_camera

# Monitor system health
vcgencmd measure_temp      # Temperature
vcgencmd measure_volts     # Voltage levels
vcgencmd get_throttled     # Throttling status

# GPIO status (for motor connections)
gpio readall
```

## 🎯 Best Practices for Robotics

### 1. Optimize for Real-time Performance
- Use 30 FPS for smooth autonomous operation
- Keep detection threshold balanced (0.5-0.7)
- Enable automatic ROI for optimal aspect ratio
- Monitor inference timing and adjust accordingly

### 2. Robust Autonomous Behavior
- Combine AI detection with traditional computer vision
- Implement safety-first logic (obstacle avoidance priority)
- Use confidence scores for decision weighting
- Add temporal filtering for stable tracking

### 3. Power Management
- Use efficient power supply (5V 3A minimum)
- Consider battery capacity for AI processing load
- Implement thermal monitoring and throttling
- Optimize processing frequency based on application needs

### 4. Environmental Considerations
- Test in various lighting conditions
- Account for motion blur during robot movement
- Consider camera mounting angle and vibration
- Implement adaptive exposure for changing conditions

## 📚 Additional Resources

- [Official Sony IMX500 Documentation](https://developer.aitrios.sony-semicon.com/en/raspberrypi-ai-camera)
- [Raspberry Pi AI Camera Guide](https://www.raspberrypi.com/documentation/accessories/ai-camera.html)
- [Picamera2 IMX500 Examples](https://github.com/raspberrypi/picamera2/tree/main/examples/imx500)
- [ChipuRobot API Reference](API_REFERENCE.md)
- [Hardware Setup Guide](HARDWARE_GUIDE.md)

## 🤝 Contributing

Found an issue or have an improvement? Please contribute to the ChipuRobot project:

1. Report issues on GitHub
2. Submit pull requests with improvements
3. Share your AI Camera configurations and optimizations
4. Help improve documentation

---

**Happy Building! 🤖📷**