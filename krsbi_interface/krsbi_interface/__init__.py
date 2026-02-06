"""
KRSBI Interface Package

High-level interface, configuration, and utilities for KRSBI-B Soccer Robot.

This package provides:
- Constants and game rules
- Utility functions
- Launch files
- Configuration parameters
- System monitoring
"""

from .constants import (
    GameState,
    SetPlay,
    RobotRole,
    TeamColor,
    PenaltyType,
    FieldDimensions,
    RobotLimits,
)

from .utils import (
    normalize_angle,
    angle_difference,
    distance_2d,
    point_in_rectangle,
    clamp,
    map_value,
)

__all__ = [
    # Constants
    'GameState',
    'SetPlay',
    'RobotRole',
    'TeamColor',
    'PenaltyType',
    'FieldDimensions',
    'RobotLimits',
    # Utils
    'normalize_angle',
    'angle_difference',
    'distance_2d',
    'point_in_rectangle',
    'clamp',
    'map_value',
]

__version__ = '1.0.0'
__author__ = 'KRSBI-B Team'
