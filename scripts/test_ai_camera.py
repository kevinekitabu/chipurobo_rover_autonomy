#!/usr/bin/env python3
"""
ChipuRobot v0.5 - AI Camera (IMX500) Test Script
Tests AI Camera functionality, model loading, and inference performance
"""

import sys
import time
import os
from typing import Dict, Any

def print_header(title: str):
    """Print formatted test section header"""
    print(f"\n{'='*60}")
    print(f"🤖 {title}")
    print(f"{'='*60}")

def print_test_result(test_name: str, passed: bool, details: str = ""):
    """Print formatted test result"""
    status = "✅ PASS" if passed else "❌ FAIL"
    print(f"   {status} {test_name}")
    if details:
        print(f"        {details}")

def test_ai_camera_detection():
    """Test 1: AI Camera Hardware Detection"""
    print_header("TEST 1: AI Camera Hardware Detection")
    
    try:
        # Test camera detection using rpicam-hello
        import subprocess
        result = subprocess.run(['rpicam-hello', '--list-cameras'], 
                              capture_output=True, text=True, timeout=10)
        
        if result.returncode == 0 and 'imx500' in result.stdout.lower():
            print_test_result("AI Camera Hardware Detection", True, 
                            "IMX500 camera detected successfully")
            return True
        else:
            print_test_result("AI Camera Hardware Detection", False, 
                            "IMX500 camera not detected")
            print(f"        Output: {result.stdout}")
            return False
            
    except subprocess.TimeoutExpired:
        print_test_result("AI Camera Hardware Detection", False, 
                        "Camera detection timed out")
        return False
    except FileNotFoundError:
        print_test_result("AI Camera Hardware Detection", False, 
                        "rpicam-hello command not found")
        return False
    except Exception as e:
        print_test_result("AI Camera Hardware Detection", False, 
                        f"Error: {e}")
        return False

def test_ai_firmware_installation():
    """Test 2: AI Firmware and Models"""
    print_header("TEST 2: AI Firmware and Models")
    
    tests_passed = 0
    total_tests = 3
    
    # Test 1: Firmware files
    firmware_files = [
        '/lib/firmware/imx500_loader.fpk',
        '/lib/firmware/imx500_firmware.fpk'
    ]
    
    for firmware_file in firmware_files:
        if os.path.exists(firmware_file):
            size = os.path.getsize(firmware_file)
            print_test_result(f"Firmware {os.path.basename(firmware_file)}", True,
                            f"Found ({size:,} bytes)")
            tests_passed += 1
        else:
            print_test_result(f"Firmware {os.path.basename(firmware_file)}", False,
                            "File not found")
    
    # Test 2: Model directory
    model_dir = '/usr/share/imx500-models'
    if os.path.exists(model_dir):
        models = [f for f in os.listdir(model_dir) if f.endswith('.rpk')]
        if models:
            print_test_result("AI Model Files", True, 
                            f"Found {len(models)} model files")
            for model in models[:3]:  # Show first 3 models
                print(f"           → {model}")
            tests_passed += 1
        else:
            print_test_result("AI Model Files", False, 
                            "No .rpk model files found")
    else:
        print_test_result("AI Model Files", False, 
                        "Model directory not found")
    
    return tests_passed == total_tests

def test_picamera2_imx500():
    """Test 3: Picamera2 and IMX500 Integration"""
    print_header("TEST 3: Picamera2 and IMX500 Integration")
    
    try:
        # Test Picamera2 import
        from picamera2 import Picamera2
        print_test_result("Picamera2 Import", True, "Module imported successfully")
        
        # Test IMX500 import
        from picamera2.devices.imx500 import IMX500
        print_test_result("IMX500 Module Import", True, "IMX500 module imported successfully")
        
        # Test IMX500 initialization with object detection model
        model_path = '/usr/share/imx500-models/imx500_network_ssd_mobilenetv2_fpnlite_320x320_pp.rpk'
        if os.path.exists(model_path):
            imx500 = IMX500(model_path)
            print_test_result("IMX500 Model Loading", True, 
                            "Object detection model loaded successfully")
            
            # Test camera initialization
            camera = Picamera2()
            config = camera.create_preview_configuration(
                main={"size": (640, 480), "format": "RGB888"}
            )
            camera.configure(config)
            print_test_result("Camera Configuration", True, 
                            "Camera configured successfully")
            
            camera.close()
            return True
        else:
            print_test_result("IMX500 Model Loading", False, 
                            f"Model file not found: {model_path}")
            return False
            
    except ImportError as e:
        print_test_result("Module Import", False, f"Import error: {e}")
        return False
    except Exception as e:
        print_test_result("IMX500 Integration", False, f"Error: {e}")
        return False

def test_chipurobo_vision():
    """Test 4: ChipuRobot Vision Integration"""
    print_header("TEST 4: ChipuRobot Vision Integration")
    
    try:
        # Import ChipuRobot vision processor
        from chipurobo.vision.camera import VisionProcessor
        print_test_result("VisionProcessor Import", True, "Module imported successfully")
        
        # Initialize vision processor
        vision = VisionProcessor()
        status = vision.get_status()
        
        # Check status components
        print_test_result("Camera Available", status['available'], 
                        f"Camera ready: {status['camera_ready']}")
        print_test_result("AI Enabled", status['ai_enabled'], 
                        f"IMX500 ready: {status['imx500_ready']}")
        print_test_result("OpenCV Available", status['opencv_available'], 
                        "OpenCV imported successfully")
        print_test_result("Picamera2 Available", status['picamera_available'], 
                        "Picamera2 imported successfully")
        
        # Test frame capture (if camera available)
        if status['available']:
            frame = vision.capture_frame()
            if frame is not None:
                print_test_result("Frame Capture", True, 
                                f"Frame size: {frame.shape}")
            else:
                print_test_result("Frame Capture", False, "No frame captured")
        
        # Test vision decision processing
        decision = vision.process_frame_for_autonomy()
        print_test_result("Vision Decision", True, 
                        f"Action: {decision.action}, Reason: {decision.reason}")
        
        # Cleanup
        vision.cleanup()
        
        return status['available'] and status['ai_enabled']
        
    except ImportError as e:
        print_test_result("ChipuRobot Import", False, f"Import error: {e}")
        return False
    except Exception as e:
        print_test_result("Vision Integration", False, f"Error: {e}")
        return False

def test_performance_benchmark():
    """Test 5: AI Performance Benchmark"""
    print_header("TEST 5: AI Performance Benchmark")
    
    try:
        from chipurobo.vision.camera import VisionProcessor
        
        vision = VisionProcessor()
        if not vision.ai_enabled:
            print_test_result("Performance Test", False, 
                            "AI not enabled - cannot benchmark")
            return False
        
        # Benchmark processing speed
        print("   Running 10-frame performance test...")
        start_time = time.time()
        
        for i in range(10):
            try:
                if vision.camera:
                    request = vision.camera.capture_request()
                    decision = vision.process_frame_for_autonomy(request)
                    request.release()
                else:
                    decision = vision.process_frame_for_autonomy()
                
                if i == 0:  # Show first decision
                    print(f"        Sample decision: {decision.action} - {decision.reason}")
                    
            except Exception as e:
                print(f"        Frame {i+1} error: {e}")
        
        end_time = time.time()
        total_time = end_time - start_time
        fps = 10 / total_time
        
        print_test_result("Performance Benchmark", True, 
                        f"Average: {fps:.1f} FPS ({total_time:.2f}s for 10 frames)")
        
        vision.cleanup()
        return fps > 5.0  # Expect at least 5 FPS
        
    except Exception as e:
        print_test_result("Performance Benchmark", False, f"Error: {e}")
        return False

def main():
    """Run all AI Camera tests"""
    print("🤖 ChipuRobot v0.5 - AI Camera (IMX500) Test Suite")
    print("   Testing Raspberry Pi AI Camera functionality and integration")
    print(f"   Python: {sys.version}")
    print(f"   Platform: {sys.platform}")
    
    # Run all tests
    tests = [
        ("AI Camera Detection", test_ai_camera_detection),
        ("Firmware Installation", test_ai_firmware_installation),
        ("Picamera2 Integration", test_picamera2_imx500),
        ("ChipuRobot Vision", test_chipurobo_vision),
        ("Performance Benchmark", test_performance_benchmark)
    ]
    
    passed_tests = 0
    total_tests = len(tests)
    
    for test_name, test_function in tests:
        try:
            if test_function():
                passed_tests += 1
        except Exception as e:
            print_test_result(test_name, False, f"Test crashed: {e}")
    
    # Print summary
    print_header("TEST RESULTS SUMMARY")
    
    if passed_tests == total_tests:
        print(f"🏆 ALL TESTS PASSED! {passed_tests}/{total_tests}")
        print("   🎉 Your AI Camera is ready for autonomous robotics!")
        print("   📚 Next steps:")
        print("      • Run: ./run.sh demo")
        print("      • Read: docs/AI_CAMERA_GUIDE.md")
        print("      • Try: ./run.sh interactive")
    else:
        print(f"⚠️  SOME TESTS FAILED: {passed_tests}/{total_tests} passed")
        print("   🔧 Troubleshooting:")
        if passed_tests == 0:
            print("      • Run: ./scripts/setup_ai_camera.sh")
            print("      • Check: Camera connection and power")
            print("      • Verify: sudo raspi-config → Interface → Camera")
        print("      • See: docs/AI_CAMERA_GUIDE.md")
        print("      • Check: docs/TROUBLESHOOTING.md")
    
    print(f"\n📊 Test completion: {passed_tests}/{total_tests} ({passed_tests/total_tests*100:.0f}%)")
    
    return passed_tests == total_tests

if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)