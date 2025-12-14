#!/usr/bin/env python3
"""
Vision Positioning System for ChipuRobo
Computer vision positioning using ArUco markers and Raspberry Pi camera
"""

import math
from typing import Optional, Tuple, Dict, Any, List

# Core imports with error handling
try:
    import RPi.GPIO as GPIO
    RPI_AVAILABLE = True
except ImportError:
    RPI_AVAILABLE = False

# Camera and vision imports
try:
    import cv2
    import picamera2
    import numpy as np
    CAMERA_AVAILABLE = True
except ImportError:
    CAMERA_AVAILABLE = False


class VisionPositioning:
    """Computer vision positioning using ArUco markers and Raspberry Pi camera"""
    
    def __init__(self):
        self.camera = None
        self.available = False
        self.aruco_dict = None
        self.aruco_params = None
        self.camera_matrix = None
        self.distortion_coeffs = None
        
        if CAMERA_AVAILABLE and RPI_AVAILABLE:
            self.setup_camera()
        else:
            print("📷 Camera running in simulation mode")
    
    def setup_camera(self) -> None:
        """Initialize Pi camera and ArUco detection"""
        try:
            self.camera = picamera2.Picamera2()
            config = self.camera.create_still_configuration(main={"size": (640, 480)})
            self.camera.configure(config)
            self.camera.start()
            
            # Setup ArUco detection
            self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
            self.aruco_params = cv2.aruco.DetectorParameters()
            
            # Default camera calibration (should be calibrated for each camera)
            self.camera_matrix = np.array([
                [640, 0, 320],
                [0, 640, 240],
                [0, 0, 1]
            ], dtype=np.float32)
            
            self.distortion_coeffs = np.zeros((4, 1))
            
            self.available = True
            print("📷 Pi Camera initialized for vision positioning")
        except Exception as e:
            print(f"❌ Camera setup failed: {e}")
    
    def capture_frame(self) -> Optional[np.ndarray]:
        """Capture a single frame"""
        if not self.available:
            return None
        
        try:
            frame = self.camera.capture_array()
            return cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        except Exception as e:
            print(f"❌ Frame capture failed: {e}")
            return None
    
    def detect_aruco_markers(self, frame: np.ndarray) -> Tuple[List, Optional[np.ndarray], List]:
        """Detect ArUco markers in frame"""
        if not self.available:
            return [], None, []
        
        try:
            gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
            detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
            corners, ids, rejected = detector.detectMarkers(gray)
            return corners, ids, rejected
        except Exception as e:
            print(f"❌ ArUco detection failed: {e}")
            return [], None, []
    
    def get_position_from_markers(self) -> Optional[Tuple[float, float, float]]:
        """Get position from ArUco markers (x, y, heading)"""
        if not self.available:
            return None
        
        frame = self.capture_frame()
        if frame is None:
            return None
        
        corners, ids, _ = self.detect_aruco_markers(frame)
        
        if ids is not None and len(ids) > 0:
            # Calculate position based on marker detection
            # This is a simplified implementation - real world would need proper calibration
            marker_center = corners[0][0].mean(axis=0)
            
            # Convert pixels to field coordinates (inches)
            # Assuming camera is mounted looking down at field
            x_pos = (marker_center[0] - 320) / 32.0  # Convert pixels to inches
            y_pos = (marker_center[1] - 240) / 32.0
            
            # Calculate heading from marker orientation
            heading = math.atan2(
                corners[0][0][1][1] - corners[0][0][0][1], 
                corners[0][0][1][0] - corners[0][0][0][0]
            )
            
            return (x_pos, y_pos, math.degrees(heading))
        
        return None
    
    def get_marker_pose(self, marker_id: int = None) -> Optional[Dict[str, Any]]:
        """Get detailed pose information for a specific marker"""
        if not self.available:
            return None
        
        frame = self.capture_frame()
        if frame is None:
            return None
        
        corners, ids, _ = self.detect_aruco_markers(frame)
        
        if ids is not None and len(ids) > 0:
            # Find specific marker or use first one
            marker_index = 0
            if marker_id is not None:
                marker_indices = np.where(ids.flatten() == marker_id)[0]
                if len(marker_indices) == 0:
                    return None
                marker_index = marker_indices[0]
            
            # Estimate pose
            try:
                marker_length = 0.1  # 10cm markers
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                    [corners[marker_index]], marker_length, 
                    self.camera_matrix, self.distortion_coeffs
                )
                
                return {
                    'id': int(ids[marker_index][0]),
                    'position': {
                        'x': float(tvecs[0][0][0]),
                        'y': float(tvecs[0][0][1]),
                        'z': float(tvecs[0][0][2])
                    },
                    'rotation': {
                        'x': float(rvecs[0][0][0]),
                        'y': float(rvecs[0][0][1]),
                        'z': float(rvecs[0][0][2])
                    },
                    'corners': corners[marker_index].tolist()
                }
            except Exception as e:
                print(f"❌ Pose estimation failed: {e}")
                return None
        
        return None
    
    def save_calibration_image(self, filename: str) -> bool:
        """Save current frame for camera calibration"""
        if not self.available:
            return False
        
        frame = self.capture_frame()
        if frame is None:
            return False
        
        try:
            cv2.imwrite(filename, cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))
            print(f"📷 Calibration image saved: {filename}")
            return True
        except Exception as e:
            print(f"❌ Failed to save calibration image: {e}")
            return False
    
    def get_status(self) -> Dict[str, Any]:
        """Get vision system status"""
        return {
            'available': self.available,
            'rpi_available': RPI_AVAILABLE,
            'camera_library_available': CAMERA_AVAILABLE,
            'aruco_dict': 'DICT_6X6_250' if self.aruco_dict is not None else None,
            'camera_resolution': (640, 480) if self.available else None,
            'calibrated': self.camera_matrix is not None
        }
    
    def cleanup(self) -> None:
        """Clean up camera resources"""
        if self.camera:
            try:
                self.camera.stop()
                print("📷 Camera cleanup completed")
            except Exception:
                pass