#!/usr/bin/env python3
"""
ChipuRobo Mission Control Backend Server
Professional Flask-based mission control server
"""

import sys
from pathlib import Path

# Add chipurobo package to path
PROJECT_ROOT = Path(__file__).parent.parent
sys.path.insert(0, str(PROJECT_ROOT))

from flask import Flask, request, jsonify
from flask_cors import CORS
import json
import os
import datetime
import threading
import time
import signal

# Import unified robot system
try:
    from chipurobo.hardware.robot import ChipuRobot
    from chipurobo.hardware.gpio_manager import GPIOPinManager
    ROBOT_AVAILABLE = True
    print("✅ ChipuRobo unified system available")
except ImportError as e:
    print(f"⚠️ ChipuRobot not available: {e}")
    ROBOT_AVAILABLE = False

app = Flask(__name__)
CORS(app)  # Enable CORS for web interface

# Data storage directories
DATA_DIR = Path(__file__).parent / "data"
DATA_DIR.mkdir(exist_ok=True)

MISSIONS_DIR = DATA_DIR / "missions"
MISSIONS_DIR.mkdir(exist_ok=True)

CONFIG_DIR = DATA_DIR / "configs"
CONFIG_DIR.mkdir(exist_ok=True)

# Current mission data and robot instance
current_mission = None
current_config = None
robot_instance = None
mission_thread = None

def check_hardware_status():
    """Check hardware availability using unified robot system"""
    hardware_status = {
        'is_raspberry_pi': False,
        'gpio_available': False,
        'platform': 'unknown',
        'gpio_error': None,
        'robot_available': ROBOT_AVAILABLE,
        'pin_assignments': {}
    }
    
    # Check if running on Raspberry Pi
    try:
        with open('/proc/cpuinfo', 'r') as f:
            cpuinfo = f.read().lower()
            if 'raspberry pi' in cpuinfo:
                hardware_status['is_raspberry_pi'] = True
                hardware_status['platform'] = 'Raspberry Pi'
    except:
        # Not on Linux or can't read cpuinfo
        import platform
        hardware_status['platform'] = platform.system()
    
    # Check robot system availability
    if ROBOT_AVAILABLE:
        try:
            import RPi.GPIO as GPIO
            hardware_status['gpio_available'] = True
        except ImportError:
            hardware_status['gpio_error'] = 'RPi.GPIO not available - simulation mode'
        
        hardware_status['pin_assignments'] = GPIOPinManager.PINS
    else:
        hardware_status['gpio_error'] = 'ChipuRobot system not available'
    
    return hardware_status

def initialize_robot():
    """Initialize the robot instance if available"""
    global robot_instance
    
    if not ROBOT_AVAILABLE:
        print("⚠️ Robot system not available")
        return False
    
    try:
        if robot_instance is None:
            config = {
                'wheelDiameter': 4.0,
                'wheelBase': 12.0,
                'encoderPPR': 11,
                'pwmFreq': 1000
            }
            robot_instance = ChipuRobot(config)
            print("✅ Robot instance initialized")
        return True
    except Exception as e:
        print(f"❌ Robot initialization failed: {e}")
        return False

@app.route('/api/status', methods=['GET'])
def get_status():
    """Get robot server status with hardware detection"""
    hardware = check_hardware_status()
    
    return jsonify({
        'status': 'online',
        'server': 'ChipuRobo Mission Control',
        'version': '1.0.0',
        'timestamp': datetime.datetime.now().isoformat(),
        'missions_stored': len(list(MISSIONS_DIR.glob('*.json'))),
        'current_mission': current_mission is not None,
        'hardware': hardware,
        'robot_ready': hardware['is_raspberry_pi'] and hardware['gpio_available']
    })

@app.route('/api/hardware', methods=['GET'])
def get_hardware_status():
    """Detailed hardware status endpoint"""
    if robot_instance:
        return jsonify(robot_instance.get_system_status())
    else:
        return jsonify(check_hardware_status())

@app.route('/api/mission/deploy', methods=['POST'])
def deploy_mission():
    """Deploy field, path, and robot config from web interface"""
    global current_mission
    
    try:
        mission_data = request.json
        
        # Generate mission ID
        mission_id = f"mission_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}"
        mission_data['missionId'] = mission_id
        
        # Save mission to file
        mission_file = MISSIONS_DIR / f"{mission_id}.json"
        with open(mission_file, 'w') as f:
            json.dump(mission_data, f, indent=2)
        
        # Set as current mission
        current_mission = mission_data
        
        # Save robot config separately
        if 'robotConfig' in mission_data:
            config_file = CONFIG_DIR / "current_robot_config.json"
            with open(config_file, 'w') as f:
                json.dump(mission_data['robotConfig'], f, indent=2)
        
        print(f"✅ Mission deployed: {mission_id}")
        print(f"   - Field: {len(mission_data.get('field', {}).get('elements', []))} elements")
        print(f"   - Path: {len(mission_data.get('path', {}).get('waypoints', []))} waypoints")
        print(f"   - Robot: {mission_data.get('robotConfig', {})}")
        
        return jsonify({
            'success': True,
            'missionId': mission_id,
            'message': 'Mission deployed successfully',
            'file': str(mission_file)
        })
        
    except Exception as e:
        print(f"❌ Deployment failed: {e}")
        return jsonify({
            'success': False,
            'error': str(e)
        }), 400

def execute_mission_thread(mission_data):
    """Execute mission in background thread"""
    global robot_instance, mission_thread
    
    try:
        if not initialize_robot():
            print("❌ Cannot execute mission - robot not available")
            return
        
        print("🚀 Starting mission execution in background...")
        robot_instance.execute_mission(mission_data)
        
    except Exception as e:
        print(f"❌ Mission execution error: {e}")
    finally:
        mission_thread = None
        print("🏁 Mission execution thread completed")

@app.route('/api/mission/execute', methods=['POST'])
def execute_current_mission():
    """Execute the current mission on the robot hardware"""
    global mission_thread
    
    if current_mission is None:
        return jsonify({
            'success': False,
            'message': 'No mission currently deployed'
        }), 404
    
    if not ROBOT_AVAILABLE:
        return jsonify({
            'success': False,
            'message': 'Robot hardware not available'
        }), 503
    
    if mission_thread and mission_thread.is_alive():
        return jsonify({
            'success': False,
            'message': 'Mission already executing'
        }), 409
    
    try:
        # Start mission execution in background thread
        mission_thread = threading.Thread(
            target=execute_mission_thread, 
            args=(current_mission,),
            name="MissionExecutor"
        )
        mission_thread.start()
        
        return jsonify({
            'success': True,
            'message': 'Mission execution started',
            'missionId': current_mission.get('missionId')
        })
        
    except Exception as e:
        return jsonify({
            'success': False,
            'error': str(e)
        }), 500

@app.route('/api/mission/stop', methods=['POST'])
def stop_mission():
    """Stop current mission execution"""
    global robot_instance
    
    if robot_instance:
        robot_instance.stop()
        print("🛑 Mission stopped")
        return jsonify({
            'success': True,
            'message': 'Mission stopped'
        })
    else:
        return jsonify({
            'success': False,
            'message': 'No robot instance available'
        }), 404

@app.route('/api/robot/status', methods=['GET'])
def get_robot_status():
    """Get real-time robot sensor data and status"""
    global robot_instance
    
    if not initialize_robot():
        return jsonify({
            'success': False,
            'message': 'Robot not available'
        }), 503
    
    try:
        sensor_data = robot_instance.get_sensor_data()
        
        return jsonify({
            'success': True,
            'robot_status': {
                'position': sensor_data['position'],
                'encoders': sensor_data['encoders'],
                'imu': sensor_data['imu'],
                'vision': sensor_data['vision'],
                'mission_running': mission_thread is not None and mission_thread.is_alive(),
                'capabilities': {
                    'motors': sensor_data['motor_driver']['initialized'],
                    'left_encoder': sensor_data['encoders']['left']['active'],
                    'right_encoder': sensor_data['encoders']['right']['active'],
                    'imu': sensor_data['imu']['available'],
                    'vision': sensor_data['vision']['available']
                }
            }
        })
        
    except Exception as e:
        return jsonify({
            'success': False,
            'error': str(e)
        }), 500

# Legacy endpoints for backward compatibility
@app.route('/status', methods=['GET'])
def legacy_status():
    """Legacy status endpoint"""
    return get_status()

@app.route('/deploy', methods=['POST'])
def legacy_deploy():
    """Legacy deploy endpoint"""
    return deploy_mission()

@app.route('/mission/execute', methods=['POST'])
def legacy_execute():
    """Legacy execute endpoint"""
    return execute_current_mission()

@app.route('/mission/stop', methods=['POST'])
def legacy_stop():
    """Legacy stop endpoint"""
    return stop_mission()

@app.route('/robot/status', methods=['GET'])
def legacy_robot_status():
    """Legacy robot status endpoint"""
    return get_robot_status()

def cleanup_robot():
    """Clean up robot resources"""
    global robot_instance
    if robot_instance:
        robot_instance.cleanup()
        robot_instance = None

def signal_handler(signum, frame):
    """Handle shutdown signals"""
    print(f"\n🛑 Shutdown signal {signum} received, cleaning up...")
    cleanup_robot()
    exit(0)

if __name__ == '__main__':
    import math
    
    # Register signal handlers for clean shutdown
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    print("🤖 ChipuRobo Mission Control Backend Server v1.0")
    print(f"📁 Data directory: {DATA_DIR.absolute()}")
    
    # Check hardware status
    hardware = check_hardware_status()
    print(f"🔧 Platform: {hardware['platform']}")
    print(f"🔌 GPIO Available: {hardware['gpio_available']}")
    print(f"🤖 Robot System: {'✅ Available' if hardware['robot_available'] else '❌ Not Available'}")
    
    if hardware['gpio_error']:
        print(f"⚠️ GPIO Status: {hardware['gpio_error']}")
    
    print(f"🚀 Starting server on http://0.0.0.0:5001")
    print("   API Endpoints:")
    print("   - GET  /api/status           - Server status")
    print("   - GET  /api/hardware         - Hardware details")
    print("   - POST /api/mission/deploy   - Deploy mission")
    print("   - POST /api/mission/execute  - Execute mission")
    print("   - POST /api/mission/stop     - Stop mission")
    print("   - GET  /api/robot/status     - Robot telemetry")
    print()
    
    try:
        app.run(host='0.0.0.0', port=5001, debug=False, threaded=True)
    except KeyboardInterrupt:
        print("\n🛑 Server stopped by user")
    finally:
        cleanup_robot()