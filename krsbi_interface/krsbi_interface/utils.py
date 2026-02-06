"""
KRSBI-B Soccer Robot - Utility Functions

Common utility functions for geometry, math, and conversions.
"""

import math
from typing import Tuple, List, Optional


# =============================================================================
# Angle Utilities
# =============================================================================

def normalize_angle(angle: float) -> float:
    """
    Normalize angle to [-π, π] range.
    
    Args:
        angle: Angle in radians
        
    Returns:
        Normalized angle in [-π, π]
    """
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


def normalize_angle_positive(angle: float) -> float:
    """
    Normalize angle to [0, 2π] range.
    
    Args:
        angle: Angle in radians
        
    Returns:
        Normalized angle in [0, 2π]
    """
    while angle >= 2 * math.pi:
        angle -= 2 * math.pi
    while angle < 0:
        angle += 2 * math.pi
    return angle


def angle_difference(angle1: float, angle2: float) -> float:
    """
    Calculate the shortest angular difference.
    
    Args:
        angle1: First angle in radians
        angle2: Second angle in radians
        
    Returns:
        Shortest difference in [-π, π]
    """
    diff = angle2 - angle1
    return normalize_angle(diff)


def degrees_to_radians(degrees: float) -> float:
    """Convert degrees to radians."""
    return degrees * math.pi / 180.0


def radians_to_degrees(radians: float) -> float:
    """Convert radians to degrees."""
    return radians * 180.0 / math.pi


# =============================================================================
# Distance & Geometry Utilities
# =============================================================================

def distance_2d(x1: float, y1: float, x2: float, y2: float) -> float:
    """
    Calculate 2D Euclidean distance.
    
    Args:
        x1, y1: First point
        x2, y2: Second point
        
    Returns:
        Distance between points
    """
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)


def distance_from_point(point1: Tuple[float, float], point2: Tuple[float, float]) -> float:
    """
    Calculate distance between two points.
    
    Args:
        point1: (x, y) tuple
        point2: (x, y) tuple
        
    Returns:
        Distance
    """
    return distance_2d(point1[0], point1[1], point2[0], point2[1])


def angle_to_point(from_x: float, from_y: float, to_x: float, to_y: float) -> float:
    """
    Calculate angle from one point to another.
    
    Args:
        from_x, from_y: Starting point
        to_x, to_y: Target point
        
    Returns:
        Angle in radians [-π, π]
    """
    return math.atan2(to_y - from_y, to_x - from_x)


def rotate_point(x: float, y: float, angle: float, 
                 cx: float = 0.0, cy: float = 0.0) -> Tuple[float, float]:
    """
    Rotate a point around a center.
    
    Args:
        x, y: Point to rotate
        angle: Rotation angle in radians
        cx, cy: Center of rotation
        
    Returns:
        Rotated point (x, y)
    """
    cos_a = math.cos(angle)
    sin_a = math.sin(angle)
    
    # Translate to origin
    tx = x - cx
    ty = y - cy
    
    # Rotate
    rx = tx * cos_a - ty * sin_a
    ry = tx * sin_a + ty * cos_a
    
    # Translate back
    return rx + cx, ry + cy


def polar_to_cartesian(distance: float, angle: float) -> Tuple[float, float]:
    """
    Convert polar coordinates to Cartesian.
    
    Args:
        distance: Distance from origin
        angle: Angle from X-axis in radians
        
    Returns:
        (x, y) Cartesian coordinates
    """
    x = distance * math.cos(angle)
    y = distance * math.sin(angle)
    return x, y


def cartesian_to_polar(x: float, y: float) -> Tuple[float, float]:
    """
    Convert Cartesian coordinates to polar.
    
    Args:
        x, y: Cartesian coordinates
        
    Returns:
        (distance, angle) polar coordinates
    """
    distance = math.sqrt(x * x + y * y)
    angle = math.atan2(y, x)
    return distance, angle


# =============================================================================
# Boundary Checks
# =============================================================================

def point_in_rectangle(x: float, y: float,
                       rect_x: float, rect_y: float,
                       rect_width: float, rect_height: float) -> bool:
    """
    Check if point is inside a rectangle.
    
    Args:
        x, y: Point to check
        rect_x, rect_y: Rectangle bottom-left corner
        rect_width, rect_height: Rectangle dimensions
        
    Returns:
        True if point is inside
    """
    return (rect_x <= x <= rect_x + rect_width and
            rect_y <= y <= rect_y + rect_height)


def point_in_circle(x: float, y: float,
                    cx: float, cy: float, radius: float) -> bool:
    """
    Check if point is inside a circle.
    
    Args:
        x, y: Point to check
        cx, cy: Circle center
        radius: Circle radius
        
    Returns:
        True if point is inside
    """
    return distance_2d(x, y, cx, cy) <= radius


def point_in_field(x: float, y: float,
                   field_length: float = 9.0,
                   field_width: float = 6.0) -> bool:
    """
    Check if point is inside the field.
    
    Args:
        x, y: Point to check
        field_length: Field length (default 9.0m)
        field_width: Field width (default 6.0m)
        
    Returns:
        True if point is inside field
    """
    half_length = field_length / 2
    half_width = field_width / 2
    return -half_length <= x <= half_length and -half_width <= y <= half_width


# =============================================================================
# Value Utilities
# =============================================================================

def clamp(value: float, min_val: float, max_val: float) -> float:
    """
    Clamp a value between min and max.
    
    Args:
        value: Value to clamp
        min_val: Minimum value
        max_val: Maximum value
        
    Returns:
        Clamped value
    """
    return max(min_val, min(value, max_val))


def map_value(value: float, 
              in_min: float, in_max: float,
              out_min: float, out_max: float) -> float:
    """
    Map a value from one range to another.
    
    Args:
        value: Input value
        in_min, in_max: Input range
        out_min, out_max: Output range
        
    Returns:
        Mapped value
    """
    # Clamp input
    value = clamp(value, in_min, in_max)
    
    # Map
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


def lerp(a: float, b: float, t: float) -> float:
    """
    Linear interpolation between two values.
    
    Args:
        a: Start value
        b: End value
        t: Interpolation factor [0, 1]
        
    Returns:
        Interpolated value
    """
    return a + (b - a) * clamp(t, 0.0, 1.0)


def sign(value: float) -> int:
    """
    Get the sign of a value.
    
    Args:
        value: Input value
        
    Returns:
        -1, 0, or 1
    """
    if value > 0:
        return 1
    elif value < 0:
        return -1
    return 0


def deadband(value: float, threshold: float) -> float:
    """
    Apply deadband to a value.
    
    Args:
        value: Input value
        threshold: Deadband threshold
        
    Returns:
        0 if |value| < threshold, else value
    """
    if abs(value) < threshold:
        return 0.0
    return value


# =============================================================================
# Velocity & Conversion Utilities
# =============================================================================

def rpm_to_rad_s(rpm: float) -> float:
    """Convert RPM to radians per second."""
    return rpm * 2 * math.pi / 60


def rad_s_to_rpm(rad_s: float) -> float:
    """Convert radians per second to RPM."""
    return rad_s * 60 / (2 * math.pi)


def mps_to_rpm(velocity_mps: float, wheel_radius: float) -> float:
    """
    Convert linear velocity (m/s) to wheel RPM.
    
    Args:
        velocity_mps: Linear velocity in m/s
        wheel_radius: Wheel radius in meters
        
    Returns:
        Wheel RPM
    """
    rad_s = velocity_mps / wheel_radius
    return rad_s_to_rpm(rad_s)


def rpm_to_mps(rpm: float, wheel_radius: float) -> float:
    """
    Convert wheel RPM to linear velocity (m/s).
    
    Args:
        rpm: Wheel RPM
        wheel_radius: Wheel radius in meters
        
    Returns:
        Linear velocity in m/s
    """
    rad_s = rpm_to_rad_s(rpm)
    return rad_s * wheel_radius


# =============================================================================
# Time Utilities
# =============================================================================

def format_duration(seconds: float) -> str:
    """
    Format duration as MM:SS string.
    
    Args:
        seconds: Duration in seconds
        
    Returns:
        Formatted string e.g. "10:30"
    """
    minutes = int(seconds) // 60
    secs = int(seconds) % 60
    return f"{minutes:02d}:{secs:02d}"


# =============================================================================
# Team/Color Utilities
# =============================================================================

def get_opponent_color(team_color: str) -> str:
    """
    Get opponent team color.
    
    Args:
        team_color: "blue" or "yellow"
        
    Returns:
        Opponent color
    """
    return "yellow" if team_color.lower() == "blue" else "blue"


def get_goal_x(team_color: str, field_length: float = 9.0) -> float:
    """
    Get X coordinate of own goal center.
    
    Args:
        team_color: "blue" or "yellow"
        field_length: Field length
        
    Returns:
        X coordinate of goal
    """
    half = field_length / 2
    # Blue team defends negative X goal
    return -half if team_color.lower() == "blue" else half


def get_opponent_goal_x(team_color: str, field_length: float = 9.0) -> float:
    """
    Get X coordinate of opponent goal center.
    
    Args:
        team_color: "blue" or "yellow"
        field_length: Field length
        
    Returns:
        X coordinate of opponent goal
    """
    return get_goal_x(get_opponent_color(team_color), field_length)
