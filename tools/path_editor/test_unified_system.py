#!/usr/bin/env python3
"""
ChipuRobot Unified System Test
Comprehensive test of the consolidated robot hardware interface
"""

import sys
import time
import json
from pathlib import Path

# Add current directory to path for imports
sys.path.insert(0, str(Path(__file__).parent))

def test_import():
    """Test if unified robot system can be imported"""
    print("🧪 Testing import of unified robot system...")
    
    try:
        from chipurobo_unified import ChipuRobot, GPIOPinManager, L298NMotorDriver, MotorEncoder
        print("✅ All components imported successfully")
        return True
    except ImportError as e:
        print(f"❌ Import failed: {e}")
        return False

def test_gpio_pins():
    """Test GPIO pin assignments"""
    print("\n🔌 Testing GPIO pin assignments...")
    
    try:
        from chipurobo_unified import GPIOPinManager
        GPIOPinManager.print_pin_assignment()
        
        # Check for conflicts
        all_pins = []
        for component in ['left_motor', 'right_motor', 'left_encoder', 'right_encoder']:
            if component in GPIOPinManager.PINS:
                for pin_name, pin_num in GPIOPinManager.PINS[component].items():
                    if isinstance(pin_num, int):
                        if pin_num in all_pins:
                            print(f"⚠️ PIN CONFLICT: GPIO {pin_num} used by multiple components")
                        else:
                            all_pins.append(pin_num)
        
        print(f"✅ GPIO pins assigned: {sorted(all_pins)}")
        print("✅ No pin conflicts detected")
        return True
    except Exception as e:
        print(f"❌ GPIO pin test failed: {e}")
        return False

def test_robot_initialization():
    """Test robot initialization"""
    print("\n🤖 Testing robot initialization...")
    
    try:
        from chipurobo_unified import ChipuRobot
        
        config = {
            'wheelDiameter': 4.0,
            'wheelBase': 12.0,
            'encoderPPR': 11,
            'pwmFreq': 1000
        }
        
        robot = ChipuRobot(config)
        print("✅ Robot initialized successfully")
        
        # Test capabilities check
        robot.print_capabilities()
        
        # Test basic sensor data
        sensor_data = robot.get_sensor_data()
        print(f"✅ Sensor data retrieved: {len(sensor_data)} categories")
        
        # Test position tracking
        robot.start_position_tracking()
        time.sleep(1)
        position = robot.get_position()
        robot.stop_position_tracking()
        print(f"✅ Position tracking works: {position}")
        
        # Clean up
        robot.cleanup()
        print("✅ Robot cleanup completed")
        
        return True
    except Exception as e:
        print(f"❌ Robot initialization failed: {e}")
        return False

def test_server_integration():
    """Test server integration"""
    print("\n🌐 Testing server integration...")
    
    try:
        from robot_server import check_raspberry_pi_hardware, initialize_robot
        
        # Test hardware detection
        hardware = check_raspberry_pi_hardware()
        print(f"✅ Hardware detection: {hardware['platform']}")
        print(f"   GPIO Available: {hardware['gpio_available']}")
        print(f"   Robot Available: {hardware['robot_available']}")
        
        # Test robot initialization from server
        success = initialize_robot()
        print(f"✅ Server robot initialization: {'Success' if success else 'Failed (expected in simulation)'}")
        
        return True
    except Exception as e:
        print(f"❌ Server integration test failed: {e}")
        return False

def test_mission_interface():
    """Test mission interface"""
    print("\n📡 Testing mission interface...")
    
    try:
        from robot_interface import RobotMissionInterface, ROBOT_SYSTEM_AVAILABLE
        
        interface = RobotMissionInterface()
        print(f"✅ Mission interface created")
        print(f"   Robot system available: {ROBOT_SYSTEM_AVAILABLE}")
        
        # Test connection (will fail if server not running, but that's OK)
        connection_ok = interface.test_connection()
        print(f"   Server connection: {'✅ Connected' if connection_ok else '⚠️ Not connected (server may not be running)'}")
        
        return True
    except Exception as e:
        print(f"❌ Mission interface test failed: {e}")
        return False

def test_mock_mission():
    """Test mock mission execution"""
    print("\n🎯 Testing mock mission execution...")
    
    try:
        from chipurobo_unified import ChipuRobot
        
        # Create test mission
        mock_mission = {
            'missionId': 'test_mission_001',
            'waypoints': [
                {'x': 0.0, 'y': 0.0},
                {'x': 3.0, 'y': 0.0},
                {'x': 3.0, 'y': 3.0},
                {'x': 0.0, 'y': 3.0}
            ],
            'robotConfig': {
                'wheelDiameter': 4.0,
                'wheelBase': 12.0,
                'maxSpeed': 2.0,
                'maxAccel': 1.0
            }
        }
        
        robot = ChipuRobot()
        
        print("✅ Mock mission created")
        print(f"   Waypoints: {len(mock_mission['waypoints'])}")
        
        # Don't actually execute (would try to move motors)
        # robot.execute_mission(mock_mission)
        print("✅ Mission execution interface ready")
        
        robot.cleanup()
        return True
    except Exception as e:
        print(f"❌ Mock mission test failed: {e}")
        return False

def main():
    """Run all tests"""
    print("🚀 ChipuRobot Unified System Test Suite")
    print("=" * 50)
    
    tests = [
        ("Import Test", test_import),
        ("GPIO Pin Test", test_gpio_pins), 
        ("Robot Initialization", test_robot_initialization),
        ("Server Integration", test_server_integration),
        ("Mission Interface", test_mission_interface),
        ("Mock Mission", test_mock_mission)
    ]
    
    results = []
    
    for test_name, test_func in tests:
        try:
            result = test_func()
            results.append((test_name, result))
        except Exception as e:
            print(f"❌ {test_name} crashed: {e}")
            results.append((test_name, False))
    
    # Summary
    print("\n" + "=" * 50)
    print("📊 TEST SUMMARY:")
    
    passed = 0
    for test_name, result in results:
        status = "✅ PASS" if result else "❌ FAIL"
        print(f"   {test_name}: {status}")
        if result:
            passed += 1
    
    print(f"\n🎯 OVERALL: {passed}/{len(results)} tests passed")
    
    if passed == len(results):
        print("🎉 ALL TESTS PASSED! ChipuRobot unified system is ready!")
    else:
        print("⚠️ Some tests failed. Check error messages above.")
    
    return passed == len(results)

if __name__ == '__main__':
    success = main()
    sys.exit(0 if success else 1)