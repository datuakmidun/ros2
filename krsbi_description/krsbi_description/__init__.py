"""
KRSBI Description Package

Robot description (URDF) for KRSBI-B Soccer Robot.

This package provides:
- URDF/Xacro model of the robot
- Launch files for visualization
- Configuration files for robot parameters
- RViz configuration for display
"""

from .state_publisher import KrsbiStatePublisher

__all__ = ['KrsbiStatePublisher']
__version__ = '1.0.0'
__author__ = 'KRSBI-B Team'
