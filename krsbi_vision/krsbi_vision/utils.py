"""
KRSBI-B Soccer Robot - Vision Utilities

Utility functions for image processing, coordinate transformation,
and fisheye unwrapping.
"""

import numpy as np
import cv2
from typing import Tuple, Optional, List
import math


# =============================================================================
# Fisheye / Omnidirectional Camera Utilities
# =============================================================================

def unwrap_fisheye(
    image: np.ndarray,
    center: Tuple[int, int],
    inner_radius: int,
    outer_radius: int,
    output_width: int = 640,
    output_height: int = 120,
    angle_offset: float = 0.0,
) -> np.ndarray:
    """
    Unwrap omnidirectional fisheye image to panoramic view.
    
    Args:
        image: Input fisheye image
        center: (cx, cy) center of omnidirectional mirror
        inner_radius: Inner radius (blind spot)
        outer_radius: Outer usable radius
        output_width: Output panorama width
        output_height: Output panorama height
        angle_offset: Rotation offset in degrees (0 = front)
        
    Returns:
        Unwrapped panoramic image
    """
    h, w = image.shape[:2]
    cx, cy = center
    
    # Create output image
    if len(image.shape) == 3:
        output = np.zeros((output_height, output_width, 3), dtype=np.uint8)
    else:
        output = np.zeros((output_height, output_width), dtype=np.uint8)
    
    # Create mapping
    angle_offset_rad = np.radians(angle_offset)
    
    for out_y in range(output_height):
        for out_x in range(output_width):
            # Map output coordinates to angle and radius
            angle = (out_x / output_width) * 2 * np.pi + angle_offset_rad
            radius = inner_radius + (out_y / output_height) * (outer_radius - inner_radius)
            
            # Convert to source coordinates
            src_x = int(cx + radius * np.cos(angle))
            src_y = int(cy + radius * np.sin(angle))
            
            # Check bounds
            if 0 <= src_x < w and 0 <= src_y < h:
                output[out_y, out_x] = image[src_y, src_x]
    
    return output


def unwrap_fisheye_fast(
    image: np.ndarray,
    map_x: np.ndarray,
    map_y: np.ndarray,
) -> np.ndarray:
    """
    Fast fisheye unwrapping using precomputed maps.
    
    Args:
        image: Input image
        map_x: X coordinate map
        map_y: Y coordinate map
        
    Returns:
        Unwrapped image
    """
    return cv2.remap(image, map_x, map_y, cv2.INTER_LINEAR)


def create_unwrap_maps(
    image_size: Tuple[int, int],
    center: Tuple[int, int],
    inner_radius: int,
    outer_radius: int,
    output_size: Tuple[int, int],
    angle_offset: float = 0.0,
) -> Tuple[np.ndarray, np.ndarray]:
    """
    Create precomputed unwrap maps for fast remapping.
    
    Args:
        image_size: (width, height) of input image
        center: (cx, cy) mirror center
        inner_radius: Inner radius
        outer_radius: Outer radius
        output_size: (width, height) of output
        angle_offset: Rotation offset in degrees
        
    Returns:
        (map_x, map_y) coordinate maps
    """
    out_w, out_h = output_size
    cx, cy = center
    
    # Create coordinate grids
    out_x = np.arange(out_w)
    out_y = np.arange(out_h)
    out_x_grid, out_y_grid = np.meshgrid(out_x, out_y)
    
    # Calculate angles and radii
    angle_offset_rad = np.radians(angle_offset)
    angles = (out_x_grid / out_w) * 2 * np.pi + angle_offset_rad
    radii = inner_radius + (out_y_grid / out_h) * (outer_radius - inner_radius)
    
    # Convert to source coordinates
    map_x = (cx + radii * np.cos(angles)).astype(np.float32)
    map_y = (cy + radii * np.sin(angles)).astype(np.float32)
    
    return map_x, map_y


def fisheye_to_cartesian(
    u: float, v: float,
    center: Tuple[int, int],
    focal_length: float,
    fov: float = 180.0,
) -> Tuple[float, float, float]:
    """
    Convert fisheye pixel coordinates to 3D direction.
    
    Args:
        u, v: Pixel coordinates
        center: Image center
        focal_length: Focal length
        fov: Field of view in degrees
        
    Returns:
        (x, y, z) unit direction vector
    """
    cx, cy = center
    dx = u - cx
    dy = v - cy
    r = np.sqrt(dx**2 + dy**2)
    
    if r < 0.001:
        return 0.0, 0.0, 1.0
    
    # Equidistant projection: r = f * theta
    theta = r / focal_length
    
    # Limit to valid range
    max_theta = np.radians(fov / 2)
    theta = min(theta, max_theta)
    
    # Convert to 3D
    phi = np.arctan2(dy, dx)
    x = np.sin(theta) * np.cos(phi)
    y = np.sin(theta) * np.sin(phi)
    z = np.cos(theta)
    
    return x, y, z


# =============================================================================
# Coordinate Transformation
# =============================================================================

def pixel_to_world(
    u: float, v: float,
    camera_matrix: np.ndarray,
    height_above_ground: float,
    camera_height: float,
    camera_pitch: float = 0.0,
) -> Tuple[float, float]:
    """
    Convert pixel coordinates to world coordinates (on ground plane).
    
    Args:
        u, v: Pixel coordinates
        camera_matrix: 3x3 intrinsic matrix [fx, 0, cx; 0, fy, cy; 0, 0, 1]
        height_above_ground: Object height above ground (0 for ground)
        camera_height: Camera height above ground
        camera_pitch: Camera pitch angle in radians (negative = looking down)
        
    Returns:
        (x, y) world coordinates in meters (robot frame)
    """
    fx = camera_matrix[0, 0]
    fy = camera_matrix[1, 1]
    cx = camera_matrix[0, 2]
    cy = camera_matrix[1, 2]
    
    # Normalize pixel coordinates
    x_norm = (u - cx) / fx
    y_norm = (v - cy) / fy
    
    # Create direction vector
    # [x_norm, y_norm, 1] is direction in camera frame
    
    # Apply pitch rotation
    cos_p = np.cos(camera_pitch)
    sin_p = np.sin(camera_pitch)
    
    # Direction in robot frame (assuming camera faces forward)
    dir_x = x_norm
    dir_y = y_norm * cos_p + sin_p
    dir_z = -y_norm * sin_p + cos_p
    
    if abs(dir_z) < 0.001:
        return float('inf'), float('inf')
    
    # Intersect with ground plane (z = height_above_ground)
    t = (height_above_ground - camera_height) / (-dir_z)
    
    if t < 0:
        return float('inf'), float('inf')
    
    world_x = t * dir_x
    world_y = t  # Forward direction
    
    return world_x, world_y


def world_to_pixel(
    x: float, y: float, z: float,
    camera_matrix: np.ndarray,
    camera_height: float,
    camera_pitch: float = 0.0,
) -> Tuple[float, float]:
    """
    Convert world coordinates to pixel coordinates.
    
    Args:
        x, y, z: World coordinates (robot frame)
        camera_matrix: 3x3 intrinsic matrix
        camera_height: Camera height
        camera_pitch: Camera pitch angle
        
    Returns:
        (u, v) pixel coordinates
    """
    fx = camera_matrix[0, 0]
    fy = camera_matrix[1, 1]
    cx = camera_matrix[0, 2]
    cy = camera_matrix[1, 2]
    
    # Transform to camera frame
    cos_p = np.cos(-camera_pitch)
    sin_p = np.sin(-camera_pitch)
    
    cam_x = x
    cam_y = y * cos_p - (z - camera_height) * sin_p
    cam_z = y * sin_p + (z - camera_height) * cos_p
    
    if cam_z <= 0:
        return -1, -1
    
    # Project
    u = fx * cam_x / cam_z + cx
    v = fy * cam_y / cam_z + cy
    
    return u, v


def estimate_distance(
    pixel_size: float,
    real_size: float,
    focal_length: float,
) -> float:
    """
    Estimate distance to object based on apparent size.
    
    Args:
        pixel_size: Object size in pixels (diameter/width)
        real_size: Real object size in meters
        focal_length: Camera focal length in pixels
        
    Returns:
        Estimated distance in meters
    """
    if pixel_size < 1:
        return float('inf')
    
    distance = (real_size * focal_length) / pixel_size
    return distance


def estimate_ball_distance(
    pixel_radius: float,
    ball_diameter: float = 0.21,
    focal_length: float = 554.0,
) -> float:
    """
    Estimate ball distance from pixel radius.
    
    Args:
        pixel_radius: Ball radius in pixels
        ball_diameter: Real ball diameter (0.21m for KRSBI)
        focal_length: Camera focal length
        
    Returns:
        Distance in meters
    """
    return estimate_distance(pixel_radius * 2, ball_diameter, focal_length)


def pixel_to_bearing(
    u: float,
    image_width: int,
    fov_horizontal: float,
) -> float:
    """
    Convert pixel x-coordinate to bearing angle.
    
    Args:
        u: Pixel x-coordinate
        image_width: Image width
        fov_horizontal: Horizontal FOV in degrees
        
    Returns:
        Bearing angle in radians (positive = left)
    """
    center_x = image_width / 2
    offset = center_x - u  # Positive if object is to the left
    
    angle = (offset / image_width) * np.radians(fov_horizontal)
    return angle


# =============================================================================
# Image Processing Utilities
# =============================================================================

def preprocess_image(
    image: np.ndarray,
    target_size: Tuple[int, int] = (416, 416),
    normalize: bool = True,
) -> np.ndarray:
    """
    Preprocess image for YOLO inference.
    
    Args:
        image: Input BGR image
        target_size: Output size (width, height)
        normalize: Normalize to [0, 1]
        
    Returns:
        Preprocessed image
    """
    # Resize
    resized = cv2.resize(image, target_size)
    
    # Convert BGR to RGB
    rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
    
    if normalize:
        # Normalize to [0, 1]
        normalized = rgb.astype(np.float32) / 255.0
        return normalized
    
    return rgb


def apply_clahe(
    image: np.ndarray,
    clip_limit: float = 2.0,
    tile_size: int = 8,
) -> np.ndarray:
    """
    Apply CLAHE (Contrast Limited Adaptive Histogram Equalization).
    
    Args:
        image: Input BGR image
        clip_limit: Contrast limit
        tile_size: Tile grid size
        
    Returns:
        Enhanced image
    """
    # Convert to LAB
    lab = cv2.cvtColor(image, cv2.COLOR_BGR2LAB)
    l, a, b = cv2.split(lab)
    
    # Apply CLAHE to L channel
    clahe = cv2.createCLAHE(clipLimit=clip_limit, tileGridSize=(tile_size, tile_size))
    l_enhanced = clahe.apply(l)
    
    # Merge and convert back
    enhanced = cv2.merge([l_enhanced, a, b])
    return cv2.cvtColor(enhanced, cv2.COLOR_LAB2BGR)


def create_mask_hsv(
    image: np.ndarray,
    hue_low: int, hue_high: int,
    sat_low: int, sat_high: int,
    val_low: int, val_high: int,
    kernel_size: int = 5,
) -> np.ndarray:
    """
    Create binary mask using HSV color thresholding.
    
    Args:
        image: Input BGR image
        hue_low/high: Hue range (0-180)
        sat_low/high: Saturation range (0-255)
        val_low/high: Value range (0-255)
        kernel_size: Morphology kernel size
        
    Returns:
        Binary mask
    """
    # Convert to HSV
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    
    # Handle hue wrap-around (red color)
    if hue_low > hue_high:
        mask1 = cv2.inRange(hsv, 
            np.array([hue_low, sat_low, val_low]),
            np.array([180, sat_high, val_high]))
        mask2 = cv2.inRange(hsv,
            np.array([0, sat_low, val_low]),
            np.array([hue_high, sat_high, val_high]))
        mask = cv2.bitwise_or(mask1, mask2)
    else:
        mask = cv2.inRange(hsv,
            np.array([hue_low, sat_low, val_low]),
            np.array([hue_high, sat_high, val_high]))
    
    # Morphological operations
    if kernel_size > 0:
        kernel = np.ones((kernel_size, kernel_size), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    
    return mask


def find_largest_contour(
    mask: np.ndarray,
    min_area: int = 100,
) -> Optional[np.ndarray]:
    """
    Find largest contour in binary mask.
    
    Args:
        mask: Binary mask
        min_area: Minimum contour area
        
    Returns:
        Largest contour or None
    """
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    if not contours:
        return None
    
    # Find largest by area
    largest = max(contours, key=cv2.contourArea)
    
    if cv2.contourArea(largest) < min_area:
        return None
    
    return largest


def fit_circle(contour: np.ndarray) -> Tuple[float, float, float]:
    """
    Fit minimum enclosing circle to contour.
    
    Args:
        contour: Input contour
        
    Returns:
        (x, y, radius) of circle
    """
    (x, y), radius = cv2.minEnclosingCircle(contour)
    return x, y, radius


def calculate_circularity(contour: np.ndarray) -> float:
    """
    Calculate circularity of contour (1.0 = perfect circle).
    
    Args:
        contour: Input contour
        
    Returns:
        Circularity value [0, 1]
    """
    area = cv2.contourArea(contour)
    perimeter = cv2.arcLength(contour, True)
    
    if perimeter == 0:
        return 0.0
    
    circularity = 4 * np.pi * area / (perimeter ** 2)
    return min(1.0, circularity)


# =============================================================================
# Drawing Utilities
# =============================================================================

def draw_detection(
    image: np.ndarray,
    bbox: Tuple[int, int, int, int],
    label: str,
    confidence: float,
    color: Tuple[int, int, int] = (0, 255, 0),
) -> np.ndarray:
    """
    Draw detection bounding box with label.
    
    Args:
        image: Input image
        bbox: (x1, y1, x2, y2) bounding box
        label: Object label
        confidence: Detection confidence
        color: Box color (BGR)
        
    Returns:
        Image with drawing
    """
    x1, y1, x2, y2 = [int(v) for v in bbox]
    
    # Draw box
    cv2.rectangle(image, (x1, y1), (x2, y2), color, 2)
    
    # Draw label
    text = f"{label}: {confidence:.2f}"
    (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
    cv2.rectangle(image, (x1, y1 - th - 4), (x1 + tw, y1), color, -1)
    cv2.putText(image, text, (x1, y1 - 2), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
    
    return image


def draw_ball(
    image: np.ndarray,
    center: Tuple[int, int],
    radius: float,
    color: Tuple[int, int, int] = (0, 165, 255),  # Orange
    distance: Optional[float] = None,
) -> np.ndarray:
    """
    Draw ball detection.
    
    Args:
        image: Input image
        center: Ball center (x, y)
        radius: Ball radius in pixels
        color: Circle color
        distance: Optional distance to display
        
    Returns:
        Image with drawing
    """
    x, y = int(center[0]), int(center[1])
    r = int(radius)
    
    # Draw circle
    cv2.circle(image, (x, y), r, color, 2)
    cv2.circle(image, (x, y), 3, (255, 255, 255), -1)
    
    # Draw distance text
    if distance is not None:
        text = f"{distance:.2f}m"
        cv2.putText(image, text, (x + r + 5, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
    
    return image


def draw_trajectory(
    image: np.ndarray,
    points: List[np.ndarray],
    color: Tuple[int, int, int] = (255, 0, 255),
) -> np.ndarray:
    """
    Draw predicted trajectory.
    
    Args:
        image: Input image
        points: List of [x, y] positions
        color: Line color
        
    Returns:
        Image with trajectory
    """
    if len(points) < 2:
        return image
    
    for i in range(len(points) - 1):
        p1 = tuple(int(v) for v in points[i])
        p2 = tuple(int(v) for v in points[i + 1])
        
        # Fade color along trajectory
        alpha = 1.0 - (i / len(points))
        c = tuple(int(v * alpha) for v in color)
        
        cv2.line(image, p1, p2, c, 2)
    
    return image
