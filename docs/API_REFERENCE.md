# ChipuRobot v0.5 - API Reference

## 📚 Module Overview

ChipuRobot v0.5 consists of three main modules:
- `chipurobo.hardware` - Motor control and robot management
- `chipurobo.vision` - Computer vision processing  
- `chipurobo.utils` - Configuration and logging utilities

---

## 🚗 Hardware Module

### `chipurobo.hardware.robot.ChipuRobot`

Main robot class that orchestrates all components.

#### Constructor
```python
ChipuRobot(config: Optional[Dict[str, Any]] = None)
```

**Parameters:**
- `config` (dict, optional): Configuration dictionary

**Example:**
```python
from chipurobo.hardware.robot import ChipuRobot

config = {
    'motor_speed': 0.8,
    'vision_mode': 'obstacle_avoidance'
}
robot = ChipuRobot(config)
```

#### Methods

##### `start_autonomous_mode() -> bool`
Starts autonomous operation using computer vision.

**Returns:** `True` if started successfully, `False` if already running

**Example:**
```python
robot.start_autonomous_mode()
# Robot now makes autonomous decisions based on camera input
```

##### `stop_autonomous_mode() -> None`
Stops autonomous operation and ensures motors are stopped.

**Example:**
```python
robot.stop_autonomous_mode()
```

##### `set_vision_mode(mode: str) -> bool`
Changes the vision processing mode.

**Parameters:**
- `mode` (str): Either "obstacle_avoidance" or "object_following"

**Returns:** `True` if mode changed successfully

**Example:**
```python
robot.set_vision_mode('object_following')
robot.set_vision_mode('obstacle_avoidance')
```

##### `manual_control(action: str, duration: float = None) -> None`
Executes manual control commands.

**Parameters:**
- `action` (str): One of "forward", "turn_left", "turn_right", "stop"
- `duration` (float, optional): Duration in seconds for timed movements

**Example:**
```python
robot.manual_control('forward', 2.0)    # Move forward for 2 seconds
robot.manual_control('turn_left')       # Turn left (default duration)
robot.manual_control('stop')            # Stop immediately
```

##### `get_robot_status() -> Dict[str, Any]`
Returns comprehensive robot status information.

**Returns:** Dictionary with robot state information

**Example:**
```python
status = robot.get_robot_status()
print(f"Autonomous: {status['autonomous_mode']}")
print(f"Vision Mode: {status['vision_status']['mode']}")
```

##### `cleanup() -> None`
Cleans up robot resources and ensures safe shutdown.

**Example:**
```python
robot.cleanup()  # Always call before program exit
```

---

### `chipurobo.hardware.motors.MotorController`

Controls differential drive motors with basic movement primitives.

#### Constructor
```python
MotorController(speed: float = 0.8)
```

**Parameters:**
- `speed` (float): Default motor speed from 0.0 to 1.0

#### Methods

##### `forward(duration: Optional[float] = None) -> None`
Moves robot forward.

**Parameters:**
- `duration` (float, optional): Time in seconds to move forward

##### `turn_left(duration: float = 0.5) -> None`
Turns robot left using differential drive.

**Parameters:**
- `duration` (float): Time in seconds to turn (default 0.5s ≈ 90°)

##### `turn_right(duration: float = 0.5) -> None` 
Turns robot right using differential drive.

##### `stop() -> None`
Stops all motors immediately.

##### `set_speed(speed: float) -> None`
Changes motor speed for all movements.

**Parameters:**
- `speed` (float): Speed from 0.0 to 1.0

##### `get_status() -> Dict[str, Any]`
Returns motor controller status.

---

## 👁️ Vision Module

### `chipurobo.vision.camera.VisionProcessor`

Processes camera input to make autonomous navigation decisions.

#### Constructor
```python
VisionProcessor()
```

Automatically initializes camera and vision processing systems.

#### Methods

##### `process_frame_for_autonomy(frame: Optional[np.ndarray] = None) -> VisionDecision`
Main processing function that returns autonomous decision.

**Parameters:**
- `frame` (numpy.ndarray, optional): Frame to process, captures new if None

**Returns:** `VisionDecision` object with recommended action

**Example:**
```python
vision = VisionProcessor()
decision = vision.process_frame_for_autonomy()

print(f"Action: {decision.action}")
print(f"Reason: {decision.reason}")
print(f"Confidence: {decision.confidence}")
```

##### `set_mode(mode: str) -> bool`
Changes vision processing mode.

**Parameters:**
- `mode` (str): "obstacle_avoidance" or "object_following"

##### `capture_frame() -> Optional[np.ndarray]`
Captures single frame from camera.

**Returns:** Image array or None if capture failed

##### `detect_obstacles(frame: np.ndarray) -> VisionDecision`
Processes frame for obstacle avoidance.

##### `detect_person_or_object(frame: np.ndarray) -> VisionDecision`
Processes frame for object following.

##### `get_status() -> Dict[str, Any]`
Returns vision processor status.

##### `cleanup() -> None`
Cleans up camera resources.

---

### `chipurobo.vision.camera.VisionDecision`

Data class representing a vision processing decision.

#### Attributes
- `action` (str): Recommended action ("forward", "turn_left", "turn_right", "stop")
- `confidence` (float): Decision confidence from 0.0 to 1.0
- `reason` (str): Human-readable explanation
- `target_detected` (bool): Whether target was detected (object following mode)
- `obstacle_detected` (bool): Whether obstacle was detected (avoidance mode)

**Example:**
```python
decision = vision.process_frame_for_autonomy()

if decision.confidence > 0.7:
    print(f"High confidence: {decision.action}")
    # Execute the action
else:
    print("Low confidence - stopping for safety")
    robot.manual_control('stop')
```

---

## 🛠️ Utils Module

### `chipurobo.utils.config_manager.load_config`

Loads configuration from TOML files.

```python
load_config(config_name: str = "development") -> Dict[str, Any]
```

**Parameters:**
- `config_name` (str): Name of config file without .toml extension

**Returns:** Configuration dictionary

**Example:**
```python
from chipurobo.utils.config_manager import load_config

config = load_config("development")
robot = ChipuRobot(config)
```

### `chipurobo.utils.logger.get_logger`

Gets configured logger instance.

```python
get_logger(name: str) -> logging.Logger
```

**Parameters:**
- `name` (str): Logger name (usually module name)

**Example:**
```python
from chipurobo.utils.logger import get_logger

logger = get_logger("MyModule")
logger.info("This is a log message")
```

---

## 🔧 Configuration Schema

### `config/development.toml`

```toml
[robot]
motor_speed = 0.8                    # Motor speed (0.0 to 1.0)
vision_mode = "obstacle_avoidance"   # Initial vision mode

[vision]
decision_interval = 0.2              # Seconds between vision decisions  
obstacle_threshold = 0.3             # Edge density threshold
target_zone_width = 100              # Pixels for "centered" target
min_target_size = 1000               # Minimum target pixels

[hardware]
# GPIO pin assignments
left_forward_pin = 17
left_backward_pin = 27  
right_forward_pin = 22
right_backward_pin = 23
left_enable_pin = 24
right_enable_pin = 25

[logging]
level = "INFO"                       # DEBUG, INFO, WARNING, ERROR
file = "robot.log"                   # Log file name
```

---

## 📝 Usage Examples

### Basic Autonomous Operation
```python
from chipurobo.hardware.robot import ChipuRobot

# Initialize robot
robot = ChipuRobot()

# Start obstacle avoidance
robot.set_vision_mode('obstacle_avoidance')
robot.start_autonomous_mode()

# Run for 30 seconds
import time
time.sleep(30)

# Clean shutdown
robot.stop_autonomous_mode()
robot.cleanup()
```

### Object Following Demo
```python
from chipurobo.hardware.robot import ChipuRobot

robot = ChipuRobot({'motor_speed': 0.6})  # Slower for following

# Switch to object following
robot.set_vision_mode('object_following')
robot.start_autonomous_mode()

# Monitor status
while True:
    status = robot.get_robot_status()
    if status['vision_status']['target_detected']:
        print("Following target...")
    else:
        print("Searching for target...")
    
    time.sleep(1)
```

### Manual Control with Vision Feedback
```python
from chipurobo.hardware.robot import ChipuRobot

robot = ChipuRobot()

# Use vision for obstacle detection but manual control
while True:
    decision = robot.vision.process_frame_for_autonomy()
    
    if decision.obstacle_detected:
        print(f"Obstacle detected: {decision.reason}")
        robot.manual_control('stop')
    else:
        # Safe to move - accept user input
        user_input = input("Command (w/a/d/s): ")
        if user_input == 'w':
            robot.manual_control('forward', 1.0)
        elif user_input == 'a':
            robot.manual_control('turn_left')
        # ... etc
```

### Custom Vision Processing
```python
from chipurobo.vision.camera import VisionProcessor
import cv2

vision = VisionProcessor()

while True:
    # Get raw frame
    frame = vision.capture_frame()
    
    # Custom processing
    gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
    edges = cv2.Canny(gray, 50, 150)
    
    # Display or save processed image
    cv2.imshow('Edge Detection', edges)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

vision.cleanup()
cv2.destroyAllWindows()
```

---

## 🚨 Error Handling

### Common Exceptions
- `ImportError`: Missing dependencies (install with pip)
- `RuntimeError`: Hardware not available (check connections)
- `ValueError`: Invalid parameters (check input values)
- `TimeoutError`: Camera/hardware not responding

### Recommended Error Handling
```python
try:
    robot = ChipuRobot()
    robot.start_autonomous_mode()
except ImportError as e:
    print(f"Missing dependency: {e}")
    print("Install with: pip install -r requirements.txt")
except RuntimeError as e:
    print(f"Hardware error: {e}")
    print("Check camera and motor connections")
except Exception as e:
    print(f"Unexpected error: {e}")
    # Always cleanup on error
    if 'robot' in locals():
        robot.cleanup()
```

---

This API reference covers all public methods and classes in ChipuRobot v0.5. For more examples, see the `examples/` and `scripts/` directories.