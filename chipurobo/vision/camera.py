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
    from picamera2.devices.imx500 import IMX500
    PICAMERA_AVAILABLE = True
    IMX500_AVAILABLE = True
except ImportError:
    PICAMERA_AVAILABLE = False
    IMX500_AVAILABLE = False
    print("📷 PiCamera2 or IMX500 not available - using simulation mode")

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
        self.imx500 = None
        self.available = False
        self.ai_enabled = False
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
        
        # AI Camera configuration
        self.ai_models = {
            'object_detection': '/usr/share/imx500-models/imx500_network_ssd_mobilenetv2_fpnlite_320x320_pp.rpk',
            'pose_estimation': '/usr/share/imx500-models/imx500_network_higherhrnet_coco_pose_640x480_pp.rpk'
        }
        self.detection_threshold = 0.5  # Confidence threshold for AI detections
        
        # Initialize camera
        if PICAMERA_AVAILABLE and OPENCV_AVAILABLE:
            self._setup_camera()
        else:
            print("📷 Vision processor running in simulation mode")
        
        print(f"👁️ Vision processor initialized - mode: {self.mode}, AI: {self.ai_enabled}")
    
    def _setup_camera(self) -> None:
        """Initialize Raspberry Pi AI Camera with IMX500 support"""
        try:
            # Initialize AI Camera if IMX500 model is available
            if IMX500_AVAILABLE and self._check_ai_models():
                print("🤖 Initializing AI Camera with IMX500 object detection...")
                # Initialize IMX500 with object detection model
                self.imx500 = IMX500(self.ai_models['object_detection'])
                self.ai_enabled = True
                
            self.camera = Picamera2()
            
            # Configure for real-time processing with AI support
            if self.ai_enabled:
                # Configure with IMX500 support
                config = self.camera.create_preview_configuration(
                    main={"size": (self.frame_width, self.frame_height), "format": "RGB888"},
                    controls={"FrameRate": 30}  # Optimize for real-time processing
                )
            else:
                # Standard configuration
                config = self.camera.create_preview_configuration(
                    main={"size": (self.frame_width, self.frame_height), "format": "RGB888"}
                )
                
            self.camera.configure(config)
            self.camera.start()
            self.available = True
            
            if self.ai_enabled:
                print("📷 Raspberry Pi AI Camera (IMX500) initialized successfully")
                # Show firmware loading progress
                self.imx500.show_network_fw_progress_bar()
            else:
                print("📷 Raspberry Pi Camera initialized (standard mode)")
                
            time.sleep(2)  # Allow camera to warm up
            
        except Exception as e:
            print(f"❌ Camera setup failed: {e}")
            print("   Falling back to standard camera mode...")
            self._setup_standard_camera()
    
    def _check_ai_models(self) -> bool:
        """Check if AI models are available"""
        import os
        for model_name, model_path in self.ai_models.items():
            if not os.path.exists(model_path):
                print(f"⚠️ AI model not found: {model_path}")
                return False
        return True
    
    def _setup_standard_camera(self) -> None:
        """Fallback to standard camera without AI features"""
        try:
            self.camera = Picamera2()
            config = self.camera.create_preview_configuration(
                main={"size": (self.frame_width, self.frame_height), "format": "RGB888"}
            )
            self.camera.configure(config)
            self.camera.start()
            self.available = True
            self.ai_enabled = False
            print("📷 Standard Raspberry Pi Camera initialized")
            time.sleep(2)
        except Exception as e:
            print(f"❌ Standard camera setup also failed: {e}")
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
    
    def ai_detect_objects(self, request) -> VisionDecision:
        """
        AI-powered object detection using IMX500 neural network
        Detects people and objects for autonomous following behavior
        """
        if not self.ai_enabled or not self.imx500:
            # Fallback to traditional vision
            return self.detect_person_or_object(request)
        
        try:
            # Get AI inference outputs
            outputs = self.imx500.get_outputs(request.get_metadata())
            if not outputs:
                return VisionDecision("stop", 0.0, "No AI inference data")
            
            # Parse detection results (MobileNet SSD format)
            boxes, scores, classes = outputs[0][0], outputs[1][0], outputs[2][0]
            
            # Filter detections above confidence threshold
            valid_detections = []
            for box, score, cls in zip(boxes, scores, classes):
                if score > self.detection_threshold:
                    # Convert from inference coordinates to image coordinates
                    obj_scaled = self.imx500.convert_inference_coords(
                        box, request.get_metadata(), self.camera
                    )
                    valid_detections.append({
                        'box': obj_scaled,
                        'confidence': score,
                        'class': int(cls),
                        'center_x': obj_scaled.x + obj_scaled.width // 2
                    })
            
            if valid_detections:
                # Find person (class 0 in COCO dataset) or any high-confidence object
                person_detections = [d for d in valid_detections if d['class'] == 0]
                target_detection = person_detections[0] if person_detections else valid_detections[0]
                
                center_x = target_detection['center_x']
                confidence = target_detection['confidence']
                class_name = self._get_class_name(target_detection['class'])
                
                # Determine action based on object position
                if center_x < self.frame_center_x - self.target_zone_width // 2:
                    return VisionDecision(
                        "turn_left", 
                        confidence, 
                        f"AI detected {class_name} on left (conf: {confidence:.2f})",
                        target_detected=True
                    )
                elif center_x > self.frame_center_x + self.target_zone_width // 2:
                    return VisionDecision(
                        "turn_right", 
                        confidence, 
                        f"AI detected {class_name} on right (conf: {confidence:.2f})",
                        target_detected=True
                    )
                else:
                    # Object centered - check distance (box size)
                    box_area = target_detection['box'].width * target_detection['box'].height
                    if box_area < 5000:  # Object far away
                        return VisionDecision(
                            "forward", 
                            confidence, 
                            f"AI detected {class_name} centered and far (conf: {confidence:.2f})",
                            target_detected=True
                        )
                    else:  # Object close
                        return VisionDecision(
                            "stop", 
                            confidence, 
                            f"AI detected {class_name} centered and close (conf: {confidence:.2f})",
                            target_detected=True
                        )
            
            # No objects detected
            return VisionDecision(
                "turn_right", 
                0.2, 
                "AI: No objects detected, searching...",
                target_detected=False
            )
            
        except Exception as e:
            print(f"❌ AI detection error: {e}")
            # Fallback to traditional vision
            frame = self.camera.capture_array() if self.available else None
            return self.detect_person_or_object(frame)
    
    def ai_detect_obstacles(self, request) -> VisionDecision:
        """
        AI-powered obstacle detection using IMX500
        Combines traditional edge detection with AI object recognition
        """
        if not self.ai_enabled or not self.imx500:
            # Fallback to traditional vision
            frame = self.camera.capture_array() if self.available else None
            return self.detect_obstacles(frame)
        
        try:
            # Get current frame for traditional analysis
            frame = self.camera.capture_array()
            
            # Run traditional obstacle detection for immediate safety
            traditional_result = self.detect_obstacles(frame)
            
            # Enhance with AI object detection for better awareness
            ai_result = self.ai_detect_objects(request)
            
            # Combine results - prioritize safety (obstacle avoidance)
            if traditional_result.obstacle_detected or traditional_result.action in ["turn_left", "turn_right"]:
                return VisionDecision(
                    traditional_result.action,
                    max(traditional_result.confidence, 0.8),
                    f"Safety priority: {traditional_result.reason}",
                    obstacle_detected=True
                )
            
            # If no immediate obstacles, use AI guidance
            if ai_result.target_detected:
                return ai_result
            
            # Default to traditional result
            return traditional_result
            
        except Exception as e:
            print(f"❌ AI obstacle detection error: {e}")
            # Fallback to safe traditional detection
            frame = self.camera.capture_array() if self.available else None
            return self.detect_obstacles(frame)
    
    def _get_class_name(self, class_id: int) -> str:
        """Get human-readable name for COCO dataset class ID"""
        # Simplified COCO class names (top classes for robotics)
        coco_classes = {
            0: "person", 1: "bicycle", 2: "car", 3: "motorcycle", 5: "bus",
            7: "truck", 15: "bench", 16: "bird", 17: "cat", 18: "dog",
            39: "bottle", 56: "chair", 57: "couch", 58: "potted plant",
            59: "bed", 60: "dining table", 62: "tv", 72: "refrigerator"
        }
        return coco_classes.get(class_id, f"object_{class_id}")
    
    def process_frame_for_autonomy(self, request=None) -> VisionDecision:
        """
        Main processing function that returns autonomous decision
        Uses AI-powered detection when available, falls back to traditional vision
        
        Args:
            request: Camera request object (for AI processing) or None for traditional processing
            
        Returns:
            VisionDecision with recommended action
        """
        if not self.available:
            return VisionDecision("stop", 0.0, "Camera not available")
        
        # Use AI-powered processing if available and request provided
        if self.ai_enabled and request is not None:
            if self.mode == "obstacle_avoidance":
                return self.ai_detect_obstacles(request)
            elif self.mode == "object_following":
                return self.ai_detect_objects(request)
            else:
                return VisionDecision("stop", 0.0, f"Unknown AI mode: {self.mode}")
        
        # Fallback to traditional vision processing
        frame = self.capture_frame()
        if frame is None:
            return VisionDecision("stop", 0.0, "No camera frame")
        
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
            "ai_enabled": self.ai_enabled,
            "mode": self.mode,
            "camera_ready": self.camera is not None,
            "imx500_ready": self.imx500 is not None,
            "frame_size": (self.frame_width, self.frame_height),
            "detection_threshold": self.detection_threshold,
            "opencv_available": OPENCV_AVAILABLE,
            "picamera_available": PICAMERA_AVAILABLE,
            "imx500_available": IMX500_AVAILABLE
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
    
