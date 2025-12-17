#!/usr/bin/env python3
"""
ChipuRobo v0.5 - Computer Vision Processor
Vision-based autonomous decision making for obstacle avoidance and object following
"""

import time
import numpy as np
from typing import Optional, Tuple, Dict, Any, List
from dataclasses import dataclass

# Camera and vision imports with fallback
try:
    import cv2
    OPENCV_AVAILABLE = True
except ImportError:
    OPENCV_AVAILABLE = False
    print("📷 OpenCV not available - vision in simulation mode")

try:
    from picamera2 import Picamera2
    PICAMERA_AVAILABLE = True
except ImportError:
    PICAMERA_AVAILABLE = False
    print("📷 PiCamera2 not available - using simulation mode")

# YOLO for person detection (optional advanced feature)
try:
    import ultralytics
    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False
    print("🤖 YOLO not available - using basic vision methods")


@dataclass
class VisionDecision:
    """Decision output from vision processing"""
    action: str  # "forward", "turn_left", "turn_right", "stop"
    confidence: float  # 0.0 to 1.0
    reason: str  # Human-readable explanation
    target_detected: bool = False
    obstacle_detected: bool = False


class VisionProcessor:
    """
    Computer Vision processor for ChipuRobo v0.5
    Handles obstacle avoidance and object following using camera input
    """
    
    def __init__(self):
        """Initialize vision processor for autonomous behavior"""
        self.camera = None
        self.available = False
        self.mode = "obstacle_avoidance"  # "obstacle_avoidance" or "object_following"
        
        # Vision parameters
        self.frame_width = 640
        self.frame_height = 480
        self.frame_center_x = self.frame_width // 2
        self.frame_center_y = self.frame_height // 2
        
        # Detection thresholds
        self.obstacle_threshold = 0.3  # Fraction of screen that triggers obstacle
        self.target_zone_width = 100   # Pixels - center zone for "following" target
        self.min_target_size = 1000    # Minimum pixels for valid target
        
        # Initialize camera
        if PICAMERA_AVAILABLE and OPENCV_AVAILABLE:
            self._setup_camera()
        else:
            print("📷 Vision processor running in simulation mode")
        
        print(f"👁️ Vision processor initialized - mode: {self.mode}")
    
    def _setup_camera(self) -> None:
        """Initialize Raspberry Pi AI Camera"""
        try:
            self.camera = Picamera2()
            # Configure for real-time processing
            config = self.camera.create_preview_configuration(
                main={"size": (self.frame_width, self.frame_height), "format": "RGB888"}
            )
            self.camera.configure(config)
            self.camera.start()
            self.available = True
            print("📷 Raspberry Pi AI Camera initialized")
            time.sleep(2)  # Allow camera to warm up
        except Exception as e:
            print(f"❌ Camera setup failed: {e}")
            self.available = False
    
    def capture_frame(self) -> Optional[np.ndarray]:
        """Capture a frame from the camera"""
        if not self.available:
            # Return simulated frame for testing
            return self._generate_simulation_frame()
        
        try:
            frame = self.camera.capture_array()
            return frame
        except Exception as e:
            print(f"❌ Frame capture failed: {e}")
            return None
    
    def _generate_simulation_frame(self) -> np.ndarray:
        """Generate simulated frame for development/testing"""
        # Create a simple test pattern
        frame = np.zeros((self.frame_height, self.frame_width, 3), dtype=np.uint8)
        
        if OPENCV_AVAILABLE:
            # Add some test patterns
            cv2.rectangle(frame, (100, 100), (200, 200), (0, 255, 0), -1)  # Green square
            cv2.circle(frame, (400, 300), 50, (255, 0, 0), -1)  # Red circle
        else:
            # Simple pattern without OpenCV
            frame[100:200, 100:200] = [0, 255, 0]  # Green square
            
        return frame
    
    def detect_obstacles(self, frame: np.ndarray) -> VisionDecision:
        """
        Detect obstacles using simple computer vision
        Returns decision for obstacle avoidance behavior
        """
        if frame is None:
            return VisionDecision("stop", 0.0, "No camera frame")
        
        # Convert to grayscale for edge detection
        gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
        
        # Apply Gaussian blur to reduce noise
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        
        # Edge detection to find obstacles
        edges = cv2.Canny(blurred, 50, 150)
        
        # Focus on lower half of image (ground level obstacles)
        roi_height = frame.shape[0] // 2
        roi = edges[roi_height:, :]
        
        # Divide frame into left, center, right sections
        width = roi.shape[1]
        section_width = width // 3
        
        left_section = roi[:, :section_width]
        center_section = roi[:, section_width:2*section_width]
        right_section = roi[:, 2*section_width:]
        
        # Count edge pixels in each section
        left_edges = np.sum(left_section) / 255
        center_edges = np.sum(center_section) / 255
        right_edges = np.sum(right_section) / 255
        
        # Calculate obstacle density
        total_pixels = section_width * roi_height
        left_density = left_edges / total_pixels
        center_density = center_edges / total_pixels
        right_density = right_edges / total_pixels
        
        # Decision logic
        if center_density > self.obstacle_threshold:
            # Obstacle ahead - turn away from denser side
            if left_density < right_density:
                return VisionDecision(
                    "turn_left", 
                    0.8, 
                    f"Obstacle ahead (density: {center_density:.2f}), less dense on left"
                )
            else:
                return VisionDecision(
                    "turn_right", 
                    0.8, 
                    f"Obstacle ahead (density: {center_density:.2f}), less dense on right"
                )
        elif left_density > self.obstacle_threshold:
            return VisionDecision(
                "turn_right", 
                0.7, 
                f"Obstacle on left (density: {left_density:.2f})"
            )
        elif right_density > self.obstacle_threshold:
            return VisionDecision(
                "turn_left", 
                0.7, 
                f"Obstacle on right (density: {right_density:.2f})"
            )
        else:
            return VisionDecision(
                "forward", 
                0.9, 
                f"Path clear (max density: {max(left_density, center_density, right_density):.2f})"
            )
    
    def detect_person_or_object(self, frame: np.ndarray) -> VisionDecision:
        """
        Detect person or colored object for following behavior
        Returns decision for object following
        """
        if frame is None:
            return VisionDecision("stop", 0.0, "No camera frame")
        
        # Method 1: Simple color-based tracking (for colored objects)
        # Look for bright/colored objects that stand out
        hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)
        
        # Define range for detecting bright objects (adjust as needed)
        # This example looks for red objects
        lower_red1 = np.array([0, 120, 70])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 120, 70])
        upper_red2 = np.array([180, 255, 255])
        
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = cv2.bitwise_or(mask1, mask2)
        
        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if contours:
            # Find largest contour (assumed to be our target)
            largest_contour = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest_contour)
            
            if area > self.min_target_size:
                # Calculate centroid
                M = cv2.moments(largest_contour)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    
                    # Determine action based on target position
                    if cx < self.frame_center_x - self.target_zone_width // 2:
                        return VisionDecision(
                            "turn_left", 
                            0.8, 
                            f"Target on left (x: {cx}, area: {int(area)})",
                            target_detected=True
                        )
                    elif cx > self.frame_center_x + self.target_zone_width // 2:
                        return VisionDecision(
                            "turn_right", 
                            0.8, 
                            f"Target on right (x: {cx}, area: {int(area)})",
                            target_detected=True
                        )
                    else:
                        # Target centered - move forward or stop based on size
                        if area < 5000:  # Target far away
                            return VisionDecision(
                                "forward", 
                                0.9, 
                                f"Target centered and far (area: {int(area)})",
                                target_detected=True
                            )
                        else:  # Target close
                            return VisionDecision(
                                "stop", 
                                0.9, 
                                f"Target centered and close (area: {int(area)})",
                                target_detected=True
                            )
        
        # No target detected - search behavior
        return VisionDecision(
            "turn_right", 
            0.3, 
            "No target detected, searching...",
            target_detected=False
        )
    
    def process_frame_for_autonomy(self, frame: Optional[np.ndarray] = None) -> VisionDecision:
        """
        Main processing function that returns autonomous decision
        
        Args:
            frame: Optional frame to process, if None captures new frame
            
        Returns:
            VisionDecision with recommended action
        """
        if frame is None:
            frame = self.capture_frame()
        
        if frame is None:
            return VisionDecision("stop", 0.0, "Camera not available")
        
        # Process based on current mode
        if self.mode == "obstacle_avoidance":
            return self.detect_obstacles(frame)
        elif self.mode == "object_following":
            return self.detect_person_or_object(frame)
        else:
            return VisionDecision("stop", 0.0, f"Unknown mode: {self.mode}")
    
    def set_mode(self, mode: str) -> bool:
        """
        Change vision processing mode
        
        Args:
            mode: "obstacle_avoidance" or "object_following"
            
        Returns:
            True if mode changed successfully
        """
        if mode in ["obstacle_avoidance", "object_following"]:
            self.mode = mode
            print(f"👁️ Vision mode changed to: {mode}")
            return True
        else:
            print(f"❌ Unknown vision mode: {mode}")
            return False
    
    def get_status(self) -> Dict[str, Any]:
        """Get current vision processor status"""
        return {
            "available": self.available,
            "mode": self.mode,
            "camera_ready": self.camera is not None,
            "frame_size": (self.frame_width, self.frame_height),
            "opencv_available": OPENCV_AVAILABLE,
            "picamera_available": PICAMERA_AVAILABLE
        }
    
    def cleanup(self):
        """Clean up camera resources"""
        if self.camera:
            try:
                self.camera.stop()
                self.camera.close()
                print("📷 Camera cleaned up")
            except Exception as e:
                print(f"⚠️ Camera cleanup error: {e}")
    
