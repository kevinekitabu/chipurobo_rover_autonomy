#!/usr/bin/env python3
"""
ChipuRobot v0.5 - Computer Vision Autonomous Rover
Main execution script for KSEF demonstration
"""

import sys
import time
import signal
from pathlib import Path

# Add project root to path
PROJECT_ROOT = Path(__file__).parent.parent
sys.path.insert(0, str(PROJECT_ROOT))

from chipurobo.hardware.robot import ChipuRobot
from chipurobo.utils.config_manager import load_config


def signal_handler(signum, frame):
    """Handle Ctrl+C gracefully"""
    print("\n🛑 Shutting down ChipuRobot v0.5...")
    if 'robot' in globals():
        robot.cleanup()
    sys.exit(0)


def print_banner():
    """Print startup banner"""
    print("=" * 60)
    print("🤖 ChipuRobo v0.5 - Computer Vision Autonomous Rover")
    print("   Built for Kenya Science & Engineering Fair (KSEF)")
    print("   Computer Vision -> Intelligence -> Movement")
    print("=" * 60)


def print_help():
    """Print control help"""
    print("\n📖 Controls:")
    print("  'o' - Switch to Obstacle Avoidance mode")
    print("  'f' - Switch to Object Following mode")
    print("  'a' - Start Autonomous mode")
    print("  's' - Stop Autonomous mode")
    print("  'w' - Manual Forward")
    print("  'a' - Manual Turn Left") 
    print("  'd' - Manual Turn Right")
    print("  'x' - Manual Stop")
    print("  'h' - Show this help")
    print("  'q' - Quit")
    print("  Enter - Start/Stop autonomous mode\n")


def main():
    """Main execution function"""
    # Set up signal handling
    signal.signal(signal.SIGINT, signal_handler)
    
    print_banner()
    
    # Load configuration
    try:
        config = load_config("development")  # Use development config by default
        print(f"✅ Loaded config: development")
    except Exception as e:
        print(f"⚠️ Config load failed, using defaults: {e}")
        config = {
            'motor_speed': 0.8,
            'vision_mode': 'obstacle_avoidance'
        }
    
    # Initialize robot
    try:
        robot = ChipuRobot(config)
    except Exception as e:
        print(f"❌ Robot initialization failed: {e}")
        return 1
    
    print_help()
    
    # Main control loop
    try:
        while True:
            # Get user input (non-blocking for autonomous mode)
            try:
                user_input = input("ChipuRobot> ").strip().lower()
            except EOFError:
                break
            
            # Process commands
            if user_input == 'q' or user_input == 'quit':
                break
            elif user_input == 'h' or user_input == 'help':
                print_help()
            elif user_input == '':
                # Enter key - toggle autonomous mode
                if robot.autonomous_mode:
                    robot.stop_autonomous_mode()
                else:
                    robot.start_autonomous_mode()
            elif user_input == 'a':
                robot.start_autonomous_mode()
            elif user_input == 's':
                robot.stop_autonomous_mode()
            elif user_input == 'o':
                robot.set_vision_mode('obstacle_avoidance')
                print("👁️ Switched to Obstacle Avoidance mode")
            elif user_input == 'f':
                robot.set_vision_mode('object_following')
                print("👁️ Switched to Object Following mode")
            elif user_input == 'w':
                robot.manual_control('forward', 1.0)
            elif user_input == 'a':
                robot.manual_control('turn_left')
            elif user_input == 'd':
                robot.manual_control('turn_right')
            elif user_input == 'x':
                robot.manual_control('stop')
            elif user_input == 'status':
                status = robot.get_robot_status()
                print(f"\n🔧 Robot Status:")
                print(f"   Autonomous: {status['autonomous_mode']}")
                print(f"   Vision Mode: {status['vision_status']['mode']}")
                print(f"   Motor Speed: {status['motor_status']['speed']:.1%}")
                print()
            else:
                print(f"⚠️ Unknown command: {user_input}")
                print("Type 'h' for help")
    
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up
        print("\n🧹 Shutting down...")
        robot.cleanup()
        print("👋 ChipuRobot v0.5 - Goodbye!")
    
    return 0


if __name__ == "__main__":
    exit_code = main()
    sys.exit(exit_code)