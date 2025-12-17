"""
ChipuRobo v0.5 - Computer Vision Autonomous Rover
=================================================

A computer vision-focused autonomous robot for educational demonstrations.
Built for Kenya Science & Engineering Fair (KSEF) 2025.

Components:
- Hardware: Motor control, differential drive
- Vision: Computer vision processing for autonomous decisions
- Utils: Configuration and logging utilities

Focus: Computer Vision → Intelligence → Movement

Author: Kevin Irungu
License: MIT
"""

__version__ = "0.5.0"
__author__ = "Kevin Irungu"

from chipurobo.hardware.robot import ChipuRobot

__all__ = ['ChipuRobot']