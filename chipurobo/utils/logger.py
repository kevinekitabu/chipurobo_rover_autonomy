#!/usr/bin/env python3
"""
Logging utility for ChipuRobot
Professional logging setup with rotation and structured output
"""

import logging
import logging.handlers
from pathlib import Path
from datetime import datetime
from typing import Optional


class RobotLogger:
    """Professional logging system for ChipuRobot"""
    
    def __init__(self, name: str = "chipurobo", log_file: Optional[str] = None, 
                 level: str = "INFO", max_size: str = "10MB", backup_count: int = 5):
        """
        Initialize robot logger
        
        Args:
            name: Logger name
            log_file: Log file path (optional)
            level: Logging level
            max_size: Max log file size
            backup_count: Number of backup files
        """
        self.logger = logging.getLogger(name)
        self.logger.setLevel(getattr(logging, level.upper()))
        
        # Clear any existing handlers
        self.logger.handlers.clear()
        
        # Create formatters
        detailed_formatter = logging.Formatter(
            '%(asctime)s | %(levelname)-8s | %(name)s | %(funcName)s:%(lineno)d | %(message)s',
            datefmt='%Y-%m-%d %H:%M:%S'
        )
        
        simple_formatter = logging.Formatter(
            '%(asctime)s | %(levelname)-8s | %(message)s',
            datefmt='%H:%M:%S'
        )
        
        # Console handler
        console_handler = logging.StreamHandler()
        console_handler.setFormatter(simple_formatter)
        self.logger.addHandler(console_handler)
        
        # File handler (if log_file specified)
        if log_file:
            log_path = Path(log_file)
            log_path.parent.mkdir(parents=True, exist_ok=True)
            
            # Parse max_size
            size_bytes = self._parse_size(max_size)
            
            file_handler = logging.handlers.RotatingFileHandler(
                log_file, maxBytes=size_bytes, backupCount=backup_count
            )
            file_handler.setFormatter(detailed_formatter)
            self.logger.addHandler(file_handler)
    
    def _parse_size(self, size_str: str) -> int:
        """Parse size string like '10MB' to bytes"""
        size_str = size_str.upper().strip()
        
        if size_str.endswith('KB'):
            return int(size_str[:-2]) * 1024
        elif size_str.endswith('MB'):
            return int(size_str[:-2]) * 1024 * 1024
        elif size_str.endswith('GB'):
            return int(size_str[:-2]) * 1024 * 1024 * 1024
        else:
            return int(size_str)  # Assume bytes
    
    def debug(self, msg: str, *args, **kwargs):
        """Log debug message"""
        self.logger.debug(msg, *args, **kwargs)
    
    def info(self, msg: str, *args, **kwargs):
        """Log info message"""
        self.logger.info(msg, *args, **kwargs)
    
    def warning(self, msg: str, *args, **kwargs):
        """Log warning message"""
        self.logger.warning(msg, *args, **kwargs)
    
    def error(self, msg: str, *args, **kwargs):
        """Log error message"""
        self.logger.error(msg, *args, **kwargs)
    
    def critical(self, msg: str, *args, **kwargs):
        """Log critical message"""
        self.logger.critical(msg, *args, **kwargs)
    
    def log_robot_status(self, robot_data: dict):
        """Log structured robot status"""
        position = robot_data.get('position', {})
        self.info(f"Robot Status - Position: ({position.get('x', 0):.2f}, {position.get('y', 0):.2f}), "
                 f"Heading: {position.get('heading', 0):.1f}°")
    
    def log_mission_event(self, event: str, mission_id: str, details: str = ""):
        """Log mission-related events"""
        self.info(f"Mission {event} - ID: {mission_id} | {details}")
    
    def log_hardware_event(self, component: str, event: str, details: str = ""):
        """Log hardware-related events"""
        self.info(f"Hardware [{component}] {event} | {details}")
    
    def log_error_with_context(self, error: Exception, context: str = ""):
        """Log error with additional context"""
        self.error(f"Error in {context}: {type(error).__name__}: {str(error)}")


# Global logger instance
_logger_instance = None

def get_logger(name: str = "chipurobo") -> RobotLogger:
    """Get global logger instance"""
    global _logger_instance
    if _logger_instance is None:
        # Try to use config for logging settings
        try:
            from .config_manager import get_config
            config = get_config()
            log_config = config.get_section('logging')
            
            _logger_instance = RobotLogger(
                name=name,
                log_file=log_config.get('file'),
                level=log_config.get('level', 'INFO'),
                max_size=log_config.get('max_size', '10MB'),
                backup_count=log_config.get('backup_count', 5)
            )
        except Exception:
            # Fallback to simple logger
            _logger_instance = RobotLogger(name=name)
    
    return _logger_instance

def setup_logging(log_file: Optional[str] = None, level: str = "INFO") -> RobotLogger:
    """Setup logging for the entire application"""
    global _logger_instance
    _logger_instance = RobotLogger("chipurobo", log_file, level)
    return _logger_instance