#!/usr/bin/env python3
"""
ChipuRobot v0.5 - KSEF 2025 Demonstration Script  
Enhanced with advanced person following capabilities for professional presentation
"""

import sys
import time
import signal
from pathlib import Path

# Add project root to path
PROJECT_ROOT = Path(__file__).parent
sys.path.insert(0, str(PROJECT_ROOT))

from chipurobo.hardware.robot import ChipuRobot
from chipurobo.utils.config_manager import load_config


def signal_handler(signum, frame):
    """Handle Ctrl+C gracefully"""
    print("\n🛑 Demo interrupted - shutting down...")
    if 'robot' in globals():
        robot.cleanup()
    sys.exit(0)


def print_demo_banner():
    """Print demo banner"""
    print("=" * 70)
    print("🏆 ChipuRobo v0.5 - KSEF Demo Sequence")
    print("    Computer Vision Autonomous Rover Demonstration")
    print("    Showcasing: Vision -> Intelligence -> Movement")
    print("=" * 70)


def countdown(seconds, message):
    """Countdown with message"""
    print(f"\n⏰ {message}")
    for i in range(seconds, 0, -1):
        print(f"   Starting in {i}...", end='\r')
        time.sleep(1)
    print("   Starting now!    ")


def demo_sequence_1_obstacle_avoidance(robot):
    """Demo 1: Obstacle Avoidance"""
    print("\n" + "="*50)
    print("🚧 DEMO 1: VISION-BASED OBSTACLE AVOIDANCE")
    print("="*50)
    print("📷 The robot uses its camera to detect obstacles")
    print("🧠 Computer vision processes the image in real-time")
    print("🎯 Robot decides: forward, turn left, turn right, or stop")
    print("💡 NO encoders, NO SLAM - pure computer vision intelligence!")
    
    countdown(5, "Starting obstacle avoidance demo...")
    
    # Set obstacle avoidance mode
    robot.set_vision_mode('obstacle_avoidance')
    
    # Run for demo duration
    robot.start_autonomous_mode()
    
    demo_duration = 30  # 30 seconds
    start_time = time.time()
    
    print(f"\n🤖 Running autonomous obstacle avoidance for {demo_duration} seconds...")
    print("📺 Watch the robot navigate around obstacles using ONLY its camera!")
    
    while time.time() - start_time < demo_duration:
        remaining = demo_duration - (time.time() - start_time)
        print(f"⏱️ Demo time remaining: {remaining:.1f}s", end='\r')
        time.sleep(1)
    
    robot.stop_autonomous_mode()
    print(f"\n✅ Demo 1 complete! The robot avoided obstacles using computer vision.")


def demo_sequence_2_person_following(robot):
    """Demo 2: Enhanced Person Following"""
    print("\n" + "="*50)
    print("👥 DEMO 2: AI-POWERED PERSON FOLLOWING")
    print("="*50)
    print("🧠 The robot uses AI to detect and follow people intelligently")
    print("📷 AI Camera (IMX500) provides real-time person detection")
    print("🎯 Decision logic: person left → turn left, person right → turn right")
    print("📐 Person centered and far → approach, close → maintain distance")
    print("🔍 Lost person → intelligent search patterns")
    
    print("\n👤 Stand in front of the robot camera for person detection")
    input("Press Enter when ready to start person following demo...")
    
    countdown(3, "Starting advanced person following...")
    
    # Run enhanced person following
    demo_duration = 45  # 45 seconds
    
    print(f"\n🎯 Running person following for {demo_duration} seconds...")
    print("👥 Move around - watch the robot intelligently track and follow you!")
    print("🎯 Notice how it maintains distance and searches when you hide!")
    
    robot.run_advanced_person_following(demo_duration)
    
    print(f"\n✅ Demo 2 complete! The robot used AI to intelligently follow people.")


def interactive_demo(robot):
    """Interactive demo for audience questions"""
    print("\n" + "="*50)
    print("🎤 INTERACTIVE DEMO - AUDIENCE Q&A")
    print("="*50)
    print("Ask questions and request demonstrations!")
    print("Available commands:")
    print("  'obs' - Obstacle avoidance demo")
    print("  'person' - Enhanced person following demo") 
    print("  'follow' - Basic object following demo")
    print("  'manual' - Manual control demonstration")
    print("  'status' - Show robot technical status")
    print("  'explain' - Explain the technology")
    print("  'quit' - End demo")
    
    while True:
        try:
            command = input("\nDemo Command> ").strip().lower()
            
            if command == 'quit' or command == 'q':
                break
            elif command == 'obs':
                robot.set_vision_mode('obstacle_avoidance')
                robot.start_autonomous_mode()
                print("🚧 Obstacle avoidance active - press Enter to stop")
                input()
                robot.stop_autonomous_mode()
            elif command == 'person':
                print("👥 Enhanced person following active - press Ctrl+C to stop")
                robot.run_advanced_person_following(60)
            elif command == 'follow':
                robot.set_vision_mode('object_following')
                robot.start_autonomous_mode()
                print("🎯 Object following active - press Enter to stop")
                input()
                robot.stop_autonomous_mode()
            elif command == 'manual':
                print("🎮 Manual control - w=forward, a=left, d=right, s=stop, q=quit")
                while True:
                    manual_input = input("Manual> ").strip().lower()
                    if manual_input == 'q':
                        break
                    elif manual_input == 'w':
                        robot.manual_control('forward', 2.0)
                    elif manual_input == 'a':
                        robot.manual_control('turn_left')
                    elif manual_input == 'd':
                        robot.manual_control('turn_right')
                    elif manual_input == 's':
                        robot.manual_control('stop')
            elif command == 'status':
                status = robot.get_robot_status()
                print(f"\n🔧 ChipuRobot v0.5 Technical Status:")
                print(f"   Platform: Raspberry Pi 5 + AI Camera")
                print(f"   Motors: 4x DC motors (2 channels)")
                print(f"   Vision: {status['vision_status']['mode']}")
                print(f"   Autonomous: {status['autonomous_mode']}")
                print(f"   Hardware: {'Real' if status['motor_status']['hardware_available'] else 'Simulation'}")
            elif command == 'explain':
                print(f"\n🧠 ChipuRobo v0.5 Technology Explanation:")
                print(f"1. 📷 Raspberry Pi AI Camera captures video at 30fps")
                print(f"2. 🔍 OpenCV processes images for edges/colors/objects")
                print(f"3. 🤖 Computer vision algorithms make decisions:")
                print(f"   • Obstacle detection using edge detection")
                print(f"   • Object tracking using color filtering")
                print(f"4. ⚡ Decision engine outputs: forward/left/right/stop")
                print(f"5. 🚗 Motor controller executes movement commands")
                print(f"6. 🔄 Loop repeats 5 times per second")
                print(f"\n💡 Key: NO encoders, NO SLAM, NO complex sensors")
                print(f"   Pure computer vision intelligence!")
        
        except KeyboardInterrupt:
            break
    
    print("🎤 Interactive demo ended")


def main():
    """Main demo execution"""
    # Set up signal handling
    signal.signal(signal.SIGINT, signal_handler)
    
    print_demo_banner()
    
    # Load configuration for demo
    try:
        config = load_config("development")
        # Demo-specific settings
        config['motor_speed'] = 0.7  # Slightly slower for demo safety
        print("✅ Demo configuration loaded")
    except Exception as e:
        print(f"⚠️ Using default demo config: {e}")
        config = {
            'motor_speed': 0.7,
            'vision_mode': 'obstacle_avoidance'
        }
    
    # Initialize robot
    try:
        robot = ChipuRobot(config)
    except Exception as e:
        print(f"❌ Robot initialization failed: {e}")
        return 1
    
    print("\n🎯 KSEF Demo Sequence Ready!")
    print("This demonstration shows computer vision driving autonomous behavior")
    
    try:
        # Run demo sequences
        demo_sequence_1_obstacle_avoidance(robot)
        
        print("\n" + "="*20)
        input("Press Enter to continue to Demo 2...")
        
        demo_sequence_2_person_following(robot)
        
        print("\n" + "="*20)
        print("🎉 Structured demos complete!")
        
        # Interactive session
        if input("Continue to interactive Q&A session? (y/n): ").lower() == 'y':
            interactive_demo(robot)
        
    except KeyboardInterrupt:
        print("\n🛑 Demo interrupted")
    finally:
        # Clean up
        print("\n🧹 Demo cleanup...")
        robot.cleanup()
        print("🏆 ChipuRobo v0.5 KSEF Demo Complete!")
        print("Thank you for watching our computer vision autonomous rover!")
    
    return 0


if __name__ == "__main__":
    exit_code = main()
    sys.exit(exit_code)