"""
KRSBI-B Vision Package

Computer vision for soccer robot with:
- Dual camera support (front + omni fisheye 360°)
- YOLOv8 object detection
- Kalman filter tracking
- Ball, robot, goal, and field detection
"""

from .kalman_filter import (
    KalmanFilter2D,
    BallTracker,
    MultiObjectTracker,
)

from .utils import (
    unwrap_fisheye,
    pixel_to_world,
    world_to_pixel,
    estimate_distance,
)

__all__ = [
    # Kalman Filter
    'KalmanFilter2D',
    'BallTracker',
    'MultiObjectTracker',
    # Utils
    'unwrap_fisheye',
    'pixel_to_world',
    'world_to_pixel',
    'estimate_distance',
]

__version__ = '1.0.0'
__author__ = 'KRSBI-B Team'
