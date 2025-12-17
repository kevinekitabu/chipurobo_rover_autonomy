#!/usr/bin/env python3
"""
ChipuRobot v0.5 - Quick Demo
Simple quick test to show the robot working
"""

import sys
import time
from pathlib import Path

PROJECT_ROOT = Path(__file__).parent
sys.path.insert(0, str(PROJECT_ROOT))

from chipurobo.hardware.robot import ChipuRobot

def main():
    print("🤖 ChipuRobot v0.5 - Quick Demo")
    print("   Computer Vision → Intelligence → Movement")
    print("=" * 50)
    
    # Initialize robot
    config = {'motor_speed': 0.8, 'vision_mode': 'obstacle_avoidance'}
    robot = ChipuRobot(config)
    
    print("\n🚧 Testing Obstacle Avoidance Mode...")
    robot.set_vision_mode('obstacle_avoidance')
    robot.start_autonomous_mode()
    time.sleep(3)
    robot.stop_autonomous_mode()
    
    print("\n🎯 Testing Object Following Mode...")
    robot.set_vision_mode('object_following')
    robot.start_autonomous_mode()
    time.sleep(3)
    robot.stop_autonomous_mode()
    
    print("\n🎮 Testing Manual Control...")
    robot.manual_control('forward', 1.0)
    robot.manual_control('turn_left')
    robot.manual_control('turn_right')
    robot.manual_control('stop')
    
    robot.cleanup()
    print("\n✅ Quick demo complete! ChipuRobot v0.5 is ready! 🚀")

if __name__ == "__main__":
    main()