#!/usr/bin/env python3
"""
ChipuRobot v0.5 - Computer Vision Autonomous Rover
Simplified robot class focused on vision-based autonomous behavior
"""

import time
import threading
from typing import Dict, Any, Optional

from .motors import MotorController
from ..vision.camera import VisionProcessor
from ..utils.logger import get_logger

# Core imports with error handling
try:
    from gpiozero import Device
    RPI_AVAILABLE = True
except ImportError:
    RPI_AVAILABLE = False


class ChipuRobot:
    """
    ChipuRobot v0.5 - Computer Vision Autonomous Rover
    Simple robot class focused on vision-based autonomous behavior
    """
    
    def __init__(self, config: Optional[Dict[str, Any]] = None):
        """
        Initialize ChipuRobot v0.5 for computer vision autonomy
        
        Args:
            config: Configuration dictionary with robot parameters
        """
        self.config = config or {}
        self.logger = get_logger("ChipuRobot")
        self.running = False
        self.autonomous_mode = False
        
        print("🤖 Initializing ChipuRobot v0.5 - Computer Vision Rover...")
        
        # Initialize motor controller
        motor_speed = self.config.get('motor_speed', 0.8)
        self.motors = MotorController(speed=motor_speed)
        
        # Initialize vision processor
        self.vision = VisionProcessor()
        
        # Set initial vision mode
        initial_mode = self.config.get('vision_mode', 'obstacle_avoidance')
        self.vision.set_mode(initial_mode)
        
        # Autonomous control thread
        self.control_thread = None
        self.last_decision_time = 0
        self.decision_interval = 0.2  # 200ms between vision decisions
        
        print("✅ ChipuRobot v0.5 initialization complete!")
        self._print_status()
    
    def _print_status(self) -> None:
        """Print robot status and capabilities"""
        print("🔧 ChipuRobot v0.5 Status:")
        motor_status = self.motors.get_status()
        vision_status = self.vision.get_status()
        
        print(f"   🚗 Motors: {'✅ Ready' if motor_status['hardware_available'] else '🔧 Simulation'}")
        print(f"   📷 Vision: {'✅ Ready' if vision_status['available'] else '🔧 Simulation'}")
        print(f"   👁️ Vision Mode: {vision_status['mode']}")
        print(f"   ⚡ Motor Speed: {motor_status['speed']:.1%}")
        
        if not motor_status['hardware_available'] or not vision_status['available']:
            print("   💡 Running in development mode - perfect for testing logic!")
    
    def start_autonomous_mode(self) -> bool:
        """
        Start autonomous operation using computer vision
        
        Returns:
            True if autonomous mode started successfully
        """
        if self.autonomous_mode:
            print("⚠️ Already in autonomous mode")
            return False
        
        self.autonomous_mode = True
        self.running = True
        
        # Start autonomous control thread
        self.control_thread = threading.Thread(target=self._autonomous_control_loop, daemon=True)
        self.control_thread.start()
        
        print(f"🚀 Autonomous mode started - {self.vision.mode}")
        return True
    
    def stop_autonomous_mode(self):
        """Stop autonomous operation"""
        self.autonomous_mode = False
        self.running = False
        
        if self.control_thread:
            self.control_thread.join(timeout=2.0)
        
        # Ensure motors are stopped
        self.motors.stop()
        print("🛑 Autonomous mode stopped")
    
    def _autonomous_control_loop(self):
        """Main autonomous control loop - runs in separate thread"""
        print("🤖 Starting autonomous control loop...")
        
        while self.running and self.autonomous_mode:
            try:
                current_time = time.time()
                
                # Rate limiting - don't process too frequently
                if current_time - self.last_decision_time < self.decision_interval:
                    time.sleep(0.05)  # Small delay
                    continue
                
                # Get vision decision with AI Camera support
                request = None
                if self.vision.ai_enabled and self.vision.camera:
                    try:
                        # Capture frame with AI processing
                        request = self.vision.camera.capture_request()
                        decision = self.vision.process_frame_for_autonomy(request, mode=self.vision.mode)
                    except Exception as e:
                        print(f"⚠️ AI Camera error, using fallback: {e}")
                        decision = self.vision.process_frame_for_autonomy(mode=self.vision.mode)
                    finally:
                        if request:
                            request.release()
                else:
                    # Traditional vision processing
                    decision = self.vision.process_frame_for_autonomy(mode=self.vision.mode)
                
                # Execute decision
                self._execute_vision_decision(decision)
                
                # Log decision (every 10th decision to avoid spam)
                if int(current_time * 5) % 10 == 0:
                    print(f"👁️ {decision.action.upper()}: {decision.reason}")
                
                self.last_decision_time = current_time
                
            except Exception as e:
                self.logger.error(f"Autonomous control error: {e}")
                print(f"❌ Autonomous control error: {e}")
                # Stop on error for safety
                self.motors.stop()
                time.sleep(0.5)
        
        # Ensure motors are stopped when loop exits
        self.motors.stop()
        print("🤖 Autonomous control loop ended")
    
    def _execute_vision_decision(self, decision):
        """
        Execute the decision from vision processing
        
        Args:
            decision: VisionDecision object with action and parameters
        """
        # Safety check
        if decision.confidence < 0.3:
            self.motors.stop()
            return
        
        # Execute the decision
        if decision.action == "forward":
            self.motors.forward()
        elif decision.action == "turn_left":
            # For object following, use shorter turns
            duration = 0.3 if self.vision.mode == "object_following" else 0.5
            self.motors.turn_left(duration)
        elif decision.action == "turn_right":
            # For object following, use shorter turns
            duration = 0.3 if self.vision.mode == "object_following" else 0.5
            self.motors.turn_right(duration)
        elif decision.action == "stop":
            self.motors.stop()
        else:
            print(f"⚠️ Unknown action: {decision.action}")
            self.motors.stop()
    
    def set_vision_mode(self, mode: str) -> bool:
        """
        Change vision processing mode
        
        Args:
            mode: "obstacle_avoidance" or "object_following"
            
        Returns:
            True if mode changed successfully
        """
        return self.vision.set_mode(mode)
    
    def manual_control(self, action: str, duration: float = None):
        """
        Manual control for testing and demos
        
        Args:
            action: "forward", "turn_left", "turn_right", "stop"
            duration: Optional duration for timed movements
        """
        if self.autonomous_mode:
            print("⚠️ Disable autonomous mode for manual control")
            return
        
        if action == "forward":
            self.motors.forward(duration)
        elif action == "turn_left":
            self.motors.turn_left(duration or 0.5)
        elif action == "turn_right":
            self.motors.turn_right(duration or 0.5)
        elif action == "stop":
            self.motors.stop()
        else:
            print(f"⚠️ Unknown manual action: {action}")
    
    def get_robot_status(self) -> Dict[str, Any]:
        """Get comprehensive robot status"""
        return {
            "autonomous_mode": self.autonomous_mode,
            "running": self.running,
            "motor_status": self.motors.get_status(),
            "vision_status": self.vision.get_status(),
            "config": self.config,
            "last_decision_time": self.last_decision_time
        }
    
    def run_advanced_person_following(self, duration: int = 60):
        """
        Run advanced person following mode with sophisticated tracking
        """
        print("🤖 Starting Advanced Person Following Mode")
        print("👥 Robot will intelligently track and follow people")
        print(f"⏱️ Running for {duration} seconds")
        
        start_time = time.time()
        last_status_time = 0
        
        try:
            while time.time() - start_time < duration:
                current_time = time.time()
                
                # Get camera frame with AI support
                request = None
                if self.vision.ai_enabled and self.vision.camera:
                    try:
                        request = self.vision.camera.capture_request()
                        decision = self.vision.process_frame_for_autonomy(request, mode="person_following")
                    except Exception as e:
                        print(f"⚠️ AI Camera error, using fallback: {e}")
                        decision = self.vision.process_frame_for_autonomy(mode="person_following")
                    finally:
                        if request:
                            request.release()
                else:
                    decision = self.vision.process_frame_for_autonomy(mode="person_following")
                
                # Execute movement decision
                self._execute_vision_decision(decision)
                
                # Status updates every 3 seconds
                if current_time - last_status_time > 3.0:
                    print(f"👥 Person Following: {decision.reason} (confidence: {decision.confidence:.2f})")
                    if hasattr(self.vision, '_person_following_state'):
                        state = self.vision._person_following_state['state']
                        print(f"   State: {state}")
                    last_status_time = current_time
                
                time.sleep(0.1)  # 10 FPS processing
                
        except KeyboardInterrupt:
            print("\n⏹️ Person following stopped by user")
        finally:
            self.stop()
            print("🤖 Advanced person following complete")
    
    def run_demo_sequence(self):
        """Enhanced demo sequence showcasing person following"""
        print("🎬 ChipuRobot v0.5 - Enhanced Person Following Demo")
        print("=" * 50)
        
        try:
            # Demo sequence
            print("\n1️⃣ System Check...")
            time.sleep(2)
            
            print("\n2️⃣ Obstacle Avoidance Demo (10 seconds)")
            self.start_autonomous_mode(mode="obstacle_avoidance")
            time.sleep(10)
            self.stop_autonomous_mode()
            
            print("\n3️⃣ Advanced Person Following Demo (30 seconds)")
            self.run_advanced_person_following(30)
            
            print("\n4️⃣ Search Pattern Demo (10 seconds)")
            print("🔍 Demonstrating search behavior when no person detected")
            self.run_advanced_person_following(10)
            
        except KeyboardInterrupt:
            print("\n⏹️ Demo stopped by user")
        finally:
            self.stop()
            print("\n✅ Demo sequence complete!")

    def cleanup(self):
        """Clean up robot resources"""
        print("🧹 Cleaning up ChipuRobot...")
        self.stop_autonomous_mode()
        self.vision.cleanup()
        print("✅ ChipuRobot cleanup complete")