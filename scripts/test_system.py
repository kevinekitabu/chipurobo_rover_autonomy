#!/usr/bin/env python3
"""
ChipuRobot v0.5 - System Test
Test script to verify all components are working correctly
"""

import sys
import time
from pathlib import Path

# Add project root to path
PROJECT_ROOT = Path(__file__).parent
sys.path.insert(0, str(PROJECT_ROOT))
# Verify path is correct
print(f"Project root: {PROJECT_ROOT}")
print(f"Python path includes: {PROJECT_ROOT in [Path(p) for p in sys.path]}")
def test_imports():
    """Test that all required modules can be imported"""
    print("🧪 Testing imports...")
    
    try:
        from chipurobo.hardware.motors import MotorController
        print("   ✅ MotorController import successful")
    except ImportError as e:
        print(f"   ❌ MotorController import failed: {e}")
        return False
    
    try:
        from chipurobo.vision.camera import VisionProcessor, VisionDecision
        print("   ✅ VisionProcessor import successful")
    except ImportError as e:
        print(f"   ❌ VisionProcessor import failed: {e}")
        return False
    
    try:
        from chipurobo.hardware.robot import ChipuRobot
        print("   ✅ ChipuRobot import successful")
    except ImportError as e:
        print(f"   ❌ ChipuRobot import failed: {e}")
        return False
    
    try:
        from chipurobo.utils.logger import get_logger
        print("   ✅ Logger import successful")
    except ImportError as e:
        print(f"   ❌ Logger import failed: {e}")
        return False
    
    try:
        from chipurobo.utils.config_manager import load_config
        print("   ✅ Config manager import successful")
    except ImportError as e:
        print(f"   ❌ Config manager import failed: {e}")
        return False
    
    return True


def test_motor_controller():
    """Test motor controller functionality"""
    print("\n🚗 Testing MotorController...")
    
    try:
        from chipurobo.hardware.motors import MotorController
        
        motors = MotorController(speed=0.5)
        print("   ✅ MotorController initialized")
        
        # Test status
        status = motors.get_status()
        print(f"   ✅ Status: {status}")
        
        # Test commands (in simulation mode)
        print("   🔧 Testing motor commands...")
        motors.stop()
        time.sleep(0.1)
        
        motors.forward(0.1)  # Brief forward
        motors.turn_left(0.1)  # Brief left turn
        motors.turn_right(0.1)  # Brief right turn
        motors.stop()
        
        print("   ✅ All motor commands executed")
        return True
        
    except Exception as e:
        print(f"   ❌ MotorController test failed: {e}")
        return False


def test_vision_processor():
    """Test vision processor functionality"""
    print("\n👁️ Testing VisionProcessor...")
    
    try:
        from chipurobo.vision.camera import VisionProcessor
        
        vision = VisionProcessor()
        print("   ✅ VisionProcessor initialized")
        
        # Test status
        status = vision.get_status()
        print(f"   ✅ Status: {status}")
        
        # Test mode switching
        vision.set_mode('obstacle_avoidance')
        print("   ✅ Switched to obstacle avoidance mode")
        
        vision.set_mode('object_following')
        print("   ✅ Switched to object following mode")
        
        # Test frame processing (will use simulation frame)
        print("   🔧 Testing vision decision making...")
        decision = vision.process_frame_for_autonomy()
        print(f"   ✅ Vision decision: {decision.action} (confidence: {decision.confidence:.2f})")
        print(f"      Reason: {decision.reason}")
        
        # Cleanup
        vision.cleanup()
        print("   ✅ Vision processor cleanup complete")
        return True
        
    except Exception as e:
        print(f"   ❌ VisionProcessor test failed: {e}")
        return False


def test_robot_integration():
    """Test full robot integration"""
    print("\n🤖 Testing ChipuRobot integration...")
    
    try:
        from chipurobo.hardware.robot import ChipuRobot
        
        # Test configuration
        config = {
            'motor_speed': 0.6,
            'vision_mode': 'obstacle_avoidance'
        }
        
        robot = ChipuRobot(config)
        print("   ✅ ChipuRobot initialized with config")
        
        # Test status
        status = robot.get_robot_status()
        print(f"   ✅ Robot status retrieved")
        
        # Test mode switching
        robot.set_vision_mode('object_following')
        print("   ✅ Vision mode switched")
        
        # Test manual control
        print("   🔧 Testing manual control...")
        robot.manual_control('forward', 0.1)
        robot.manual_control('stop')
        print("   ✅ Manual control working")
        
        # Test brief autonomous mode
        print("   🚀 Testing autonomous mode (3 seconds)...")
        robot.start_autonomous_mode()
        time.sleep(3)
        robot.stop_autonomous_mode()
        print("   ✅ Autonomous mode test complete")
        
        # Cleanup
        robot.cleanup()
        print("   ✅ Robot cleanup complete")
        return True
        
    except Exception as e:
        print(f"   ❌ ChipuRobot test failed: {e}")
        return False


def test_config_loading():
    """Test configuration loading"""
    print("\n⚙️ Testing configuration loading...")
    
    try:
        from chipurobo.utils.config_manager import load_config
        
        # Test loading development config
        config = load_config("development")
        print("   ✅ Development config loaded")
        print(f"   📋 Config keys: {list(config.keys())}")
        return True
        
    except Exception as e:
        print(f"   ❌ Config loading failed: {e}")
        return False


def main():
    """Run all tests"""
    print("=" * 60)
    print("🧪 ChipuRobot v0.5 - System Test Suite")
    print("   Testing computer vision autonomous rover components")
    print("=" * 60)
    
    tests = [
        ("Import Tests", test_imports),
        ("MotorController Tests", test_motor_controller), 
        ("VisionProcessor Tests", test_vision_processor),
        ("Configuration Tests", test_config_loading),
        ("Robot Integration Tests", test_robot_integration)
    ]
    
    passed = 0
    total = len(tests)
    
    for test_name, test_func in tests:
        print(f"\n🔍 Running {test_name}...")
        if test_func():
            passed += 1
            print(f"✅ {test_name} PASSED")
        else:
            print(f"❌ {test_name} FAILED")
    
    print("\n" + "=" * 60)
    print(f"🏆 TEST RESULTS: {passed}/{total} tests passed")
    
    if passed == total:
        print("🎉 All tests passed! ChipuRobot v0.5 is ready for KSEF!")
        print("   Run 'python3 main_v05.py' for interactive mode")
        print("   Run 'python3 ksef_demo.py' for automated demo")
    else:
        print("⚠️ Some tests failed. Check the error messages above.")
        return 1
    
    return 0


if __name__ == "__main__":
    exit_code = main()
    sys.exit(exit_code)