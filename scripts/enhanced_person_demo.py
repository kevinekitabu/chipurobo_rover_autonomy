#!/usr/bin/env python3
"""
ChipuRobot v0.5 - Enhanced Person Following Demo
Demonstrates advanced person detection and following capabilities
"""

import sys
import time
from pathlib import Path

# Add project root to path
PROJECT_ROOT = Path(__file__).parent.parent
sys.path.insert(0, str(PROJECT_ROOT))

from chipurobo.hardware.robot import ChipuRobot
from chipurobo.utils.config_manager import load_config

def interactive_person_demo():
    """Interactive demonstration of enhanced person following"""
    config = load_config()
    robot = ChipuRobot(config)
    
    print("🤖 ChipuRobot v0.5 - Enhanced Person Following")
    print("=" * 50)
    print("Available modes:")
    print("1. Basic person following (10s)")
    print("2. Advanced person following (30s)")  
    print("3. Person search demo (15s)")
    print("4. Full person interaction demo (60s)")
    print("5. Continuous following (until Ctrl+C)")
    print("q. Quit")
    
    try:
        while True:
            choice = input("\nEnter your choice (1-5, q): ").strip().lower()
            
            if choice == '1':
                print("\n🎯 Basic Person Following Demo")
                robot.vision.set_mode("object_following")
                robot.start_autonomous_mode()
                time.sleep(10)
                robot.stop_autonomous_mode()
                
            elif choice == '2':
                print("\n👥 Advanced Person Following Demo")
                robot.run_advanced_person_following(30)
                
            elif choice == '3':
                print("\n🔍 Person Search Demo")
                # Set robot to search mode by ensuring no person is detected initially
                robot.run_advanced_person_following(15)
                
            elif choice == '4':
                print("\n🎬 Full Person Interaction Demo")
                robot.run_demo_sequence()
                
            elif choice == '5':
                print("\n♾️ Continuous Person Following")
                print("Press Ctrl+C to stop")
                robot.run_advanced_person_following(3600)  # 1 hour max
                
            elif choice == 'q':
                break
                
            else:
                print("❌ Invalid choice. Please try again.")
                
    except KeyboardInterrupt:
        print("\n⏹️ Demo stopped by user")
    finally:
        robot.cleanup()
        print("👋 Enhanced person following demo complete!")

if __name__ == "__main__":
    interactive_person_demo()