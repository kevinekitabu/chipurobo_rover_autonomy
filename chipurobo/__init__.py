"""
ChipuRobo - Autonomous Robot Control System
==========================================

A professional robotics framework for autonomous navigation and mission control.

Components:
- Hardware: Motor drivers, sensors, GPIO management
- Control: Navigation, path planning, autonomous driving
- Mission: Mission planning and execution system  
- Vision: Computer vision and positioning
- Web: Mission control interface
- Server: Backend API and robot communication

Author: Kevin Irungu
License: MIT
"""

__version__ = "1.0.0"
__author__ = "Kevin Irungu"

from chipurobo.hardware.robot import ChipuRobot
from chipurobo.hardware.gpio_manager import GPIOPinManager

__all__ = ['ChipuRobot', 'GPIOPinManager']