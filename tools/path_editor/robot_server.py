#!/usr/bin/env python3
"""
ChipuRobo Mission Control Backend Server
Receives field configurations, paths, and robot settings from the web interface
Provides API for robot to retrieve mission data
"""

from flask import Flask, request, jsonify
from flask_cors import CORS
import json
import os
import datetime
from pathlib import Path

app = Flask(__name__)
CORS(app)  # Enable CORS for web interface

# Data storage directories
DATA_DIR = Path(__file__).parent / "robot_data"
DATA_DIR.mkdir(exist_ok=True)

MISSIONS_DIR = DATA_DIR / "missions"
MISSIONS_DIR.mkdir(exist_ok=True)

CONFIG_DIR = DATA_DIR / "configs"
CONFIG_DIR.mkdir(exist_ok=True)

# Current mission data
current_mission = None
current_config = None

def check_raspberry_pi_hardware():
    """Check if running on Raspberry Pi with GPIO access"""
    hardware_status = {
        'is_raspberry_pi': False,
        'gpio_available': False,
        'platform': 'unknown',
        'gpio_error': None
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
    
    # Check GPIO availability
    if hardware_status['is_raspberry_pi']:
        try:
            import RPi.GPIO as GPIO
            # Try to set GPIO mode (doesn't actually configure pins)
            GPIO.setmode(GPIO.BCM)
            GPIO.setwarnings(False)
            hardware_status['gpio_available'] = True
        except ImportError:
            hardware_status['gpio_error'] = 'RPi.GPIO not installed'
        except Exception as e:
            hardware_status['gpio_error'] = str(e)
    else:
        hardware_status['gpio_error'] = 'Not running on Raspberry Pi'
    
    return hardware_status

@app.route('/status', methods=['GET'])
def get_status():
    """Get robot server status with hardware detection"""
    hardware = check_raspberry_pi_hardware()
    
    return jsonify({
        'status': 'online',
        'server': 'ChipuRobo Mission Control',
        'version': '1.0',
        'timestamp': datetime.datetime.now().isoformat(),
        'missions_stored': len(list(MISSIONS_DIR.glob('*.json'))),
        'current_mission': current_mission is not None,
        'hardware': hardware,
        'robot_ready': hardware['is_raspberry_pi'] and hardware['gpio_available']
    })

@app.route('/hardware', methods=['GET'])
def get_hardware_status():
    """Dedicated endpoint for hardware status check"""
    return jsonify(check_raspberry_pi_hardware())

@app.route('/deploy', methods=['POST'])
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

@app.route('/mission/current', methods=['GET'])
def get_current_mission():
    """Get current mission data for robot"""
    if current_mission is None:
        return jsonify({
            'success': False,
            'message': 'No mission currently deployed'
        }), 404
    
    return jsonify({
        'success': True,
        'mission': current_mission
    })

@app.route('/mission/<mission_id>', methods=['GET'])
def get_mission(mission_id):
    """Get specific mission by ID"""
    mission_file = MISSIONS_DIR / f"{mission_id}.json"
    
    if not mission_file.exists():
        return jsonify({
            'success': False,
            'message': f'Mission {mission_id} not found'
        }), 404
    
    with open(mission_file, 'r') as f:
        mission_data = json.load(f)
    
    return jsonify({
        'success': True,
        'mission': mission_data
    })

@app.route('/missions', methods=['GET'])
def list_missions():
    """List all available missions"""
    missions = []
    
    for mission_file in MISSIONS_DIR.glob('*.json'):
        try:
            with open(mission_file, 'r') as f:
                mission_data = json.load(f)
                missions.append({
                    'id': mission_data.get('missionId'),
                    'timestamp': mission_data.get('timestamp'),
                    'file': mission_file.name,
                    'waypoints': len(mission_data.get('path', {}).get('waypoints', [])),
                    'field_elements': len(mission_data.get('field', {}).get('elements', []))
                })
        except Exception as e:
            print(f"Error reading mission file {mission_file}: {e}")
    
    return jsonify({
        'success': True,
        'missions': missions
    })

@app.route('/robot/config', methods=['GET'])
def get_robot_config():
    """Get current robot configuration"""
    config_file = CONFIG_DIR / "current_robot_config.json"
    
    if not config_file.exists():
        return jsonify({
            'success': False,
            'message': 'No robot configuration found'
        }), 404
    
    with open(config_file, 'r') as f:
        config_data = json.load(f)
    
    return jsonify({
        'success': True,
        'config': config_data
    })

@app.route('/path/generate', methods=['POST'])
def generate_trajectory():
    """Generate trajectory from waypoints using robot constraints"""
    try:
        data = request.json
        waypoints = data.get('waypoints', [])
        robot_config = data.get('robotConfig', {})
        
        # This is where you'd implement trajectory generation
        # For now, return the waypoints with timing information
        
        max_speed = robot_config.get('maxSpeed', 6.0)  # ft/s
        max_accel = robot_config.get('maxAccel', 8.0)  # ft/s²
        
        trajectory = []
        total_time = 0.0
        
        for i, waypoint in enumerate(waypoints):
            if i == 0:
                # Starting point
                trajectory.append({
                    'x': waypoint['x'],
                    'y': waypoint['y'],
                    'time': 0.0,
                    'velocity': 0.0,
                    'heading': 0.0
                })
            else:
                # Calculate distance to previous waypoint
                prev = waypoints[i-1]
                dx = waypoint['x'] - prev['x']
                dy = waypoint['y'] - prev['y']
                distance = (dx**2 + dy**2)**0.5
                
                # Simple time calculation (can be improved with proper trajectory planning)
                segment_time = distance / (max_speed * 0.8)  # Use 80% of max speed
                total_time += segment_time
                
                # Calculate heading
                heading = math.atan2(dy, dx) * 180 / math.pi
                
                trajectory.append({
                    'x': waypoint['x'],
                    'y': waypoint['y'],
                    'time': total_time,
                    'velocity': max_speed * 0.8,
                    'heading': heading
                })
        
        return jsonify({
            'success': True,
            'trajectory': trajectory,
            'total_time': total_time,
            'total_distance': sum([
                ((trajectory[i]['x'] - trajectory[i-1]['x'])**2 + 
                 (trajectory[i]['y'] - trajectory[i-1]['y'])**2)**0.5 
                for i in range(1, len(trajectory))
            ])
        })
        
    except Exception as e:
        return jsonify({
            'success': False,
            'error': str(e)
        }), 400

if __name__ == '__main__':
    print("🤖 ChipuRobo Mission Control Backend Server")
    print(f"📁 Data directory: {DATA_DIR.absolute()}")
    print(f"🚀 Starting server on http://0.0.0.0:5001")
    print("   - Web interface can connect to deploy missions")
    print("   - Robot can connect to retrieve mission data")
    print()
    
    # Import math for trajectory calculations
    import math
    
    app.run(host='0.0.0.0', port=5001, debug=True)