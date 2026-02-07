#!/usr/bin/env python3
"""
KRSBI-B Soccer Robot - Ball Detector Node

Combines YOLO and color-based detection with Kalman filter tracking.

Subscribes:
    - /krsbi/camera/front/image_raw (sensor_msgs/Image)
    - /krsbi/camera/omni/image_unwrapped (sensor_msgs/Image)

Publishes:
    - /krsbi/vision/ball (krsbi_msgs/BallPosition)
    - /krsbi/vision/ball/debug (sensor_msgs/Image)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point, PointStamped
from std_msgs.msg import Header

import cv2
import numpy as np
from cv_bridge import CvBridge
import threading
import time
from typing import Optional, Tuple, List
from dataclasses import dataclass

from .kalman_filter import BallTracker
from .utils import (
    create_mask_hsv, find_largest_contour, fit_circle,
    calculate_circularity, estimate_ball_distance,
    draw_ball, draw_trajectory, apply_clahe, draw_detection,
)

# Try to import YOLO
try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False

# Try to import krsbi_msgs
try:
    from krsbi_msgs.msg import BallPosition
    MSGS_AVAILABLE = True
except ImportError:
    MSGS_AVAILABLE = False


@dataclass
class BallDetection:
    """Ball detection result."""
    x: float
    y: float
    radius: float
    confidence: float
    source: str  # 'yolo', 'color', 'front', 'omni'
    distance: float = 0.0
    bearing: float = 0.0


class BallDetectorNode(Node):
    """
    Ball detector with YOLO + color detection and Kalman filter tracking.
    """
    
    def __init__(self):
        super().__init__('ball_detector')
        
        # =================================================================
        # Parameters
        # =================================================================
        # Detection method
        self.declare_parameter('use_yolo', True)
        self.declare_parameter('use_color', True)
        self.declare_parameter('yolo_model', 'yolov8s.pt')
        self.declare_parameter('yolo_confidence', 0.5)
        self.declare_parameter('ball_class_id', 0)  # Custom model: 0, COCO: 32
        
        # Color detection (HSV) - Pink/Magenta ball
        # Pink hue wraps around: 150-180 OR 0-10
        self.declare_parameter('hue_low', 140)
        self.declare_parameter('hue_high', 180)
        self.declare_parameter('hue_low2', 0)  # Second range for wraparound
        self.declare_parameter('hue_high2', 15)
        self.declare_parameter('saturation_low', 50)
        self.declare_parameter('saturation_high', 255)
        self.declare_parameter('value_low', 50)
        self.declare_parameter('value_high', 255)
        
        # Size filtering
        self.declare_parameter('min_area', 100)
        self.declare_parameter('max_area', 50000)
        self.declare_parameter('min_circularity', 0.6)
        
        # Distance estimation
        self.declare_parameter('ball_diameter', 0.15) # FIFA Size 1
        
        # Calibration Parameters
        self.declare_parameter('focal_length_front', 554.0)
        self.declare_parameter('focal_length_omni', 122.6)    # Recalibrated (0.6m -> 0.25m w/ 51.1)
        
        # Kalman filter
        self.declare_parameter('kalman_enabled', True)
        self.declare_parameter('kalman_dt', 0.02)
        self.declare_parameter('kalman_friction', 0.3)
        
        # Debug
        self.declare_parameter('publish_debug', True)
        self.declare_parameter('detection_rate', 30.0)
        
        # Get parameters
        self.use_yolo = self.get_parameter('use_yolo').value
        self.use_color = self.get_parameter('use_color').value
        self.yolo_model = self.get_parameter('yolo_model').value
        self.yolo_conf = self.get_parameter('yolo_confidence').value
        self.ball_class_id = self.get_parameter('ball_class_id').value
        
        self.hue_low = self.get_parameter('hue_low').value
        self.hue_high = self.get_parameter('hue_high').value
        self.hue_low2 = self.get_parameter('hue_low2').value
        self.hue_high2 = self.get_parameter('hue_high2').value
        self.sat_low = self.get_parameter('saturation_low').value
        self.sat_high = self.get_parameter('saturation_high').value
        self.val_low = self.get_parameter('value_low').value
        self.val_high = self.get_parameter('value_high').value
        
        self.min_area = self.get_parameter('min_area').value
        self.max_area = self.get_parameter('max_area').value
        self.min_circularity = self.get_parameter('min_circularity').value
        
        self.ball_diameter = self.get_parameter('ball_diameter').value
        self.focal_length_front = self.get_parameter('focal_length_front').value
        self.focal_length_omni = self.get_parameter('focal_length_omni').value
        
        self.kalman_enabled = self.get_parameter('kalman_enabled').value
        self.publish_debug = self.get_parameter('publish_debug').value
        self.detection_rate = self.get_parameter('detection_rate').value
        
        # =================================================================
        # State
        # =================================================================
        self.bridge = CvBridge()
        self.model: Optional[YOLO] = None
        self.is_running = True
        
        # Frame buffers
        self.front_frame: Optional[np.ndarray] = None
        self.omni_frame: Optional[np.ndarray] = None
        self.lock = threading.Lock()
        
        # Kalman filter tracker
        if self.kalman_enabled:
            self.tracker = BallTracker(
                dt=self.get_parameter('kalman_dt').value,
                friction=self.get_parameter('kalman_friction').value,
            )
        else:
            self.tracker = None
        
        # Last detection
        self.last_detection: Optional[BallDetection] = None
        self.last_position: Optional[Tuple[float, float]] = None
        self.last_velocity: Optional[Tuple[float, float]] = None
        
        # =================================================================
        # Load YOLO model
        # =================================================================
        if self.use_yolo and YOLO_AVAILABLE:
            self.load_yolo()
        
        # =================================================================
        # QoS
        # =================================================================
        image_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # =================================================================
        # Subscribers
        # =================================================================
        self.front_sub = self.create_subscription(
            Image, '/krsbi/camera/front/image_raw',
            self.front_callback, image_qos)
        
        self.omni_sub = self.create_subscription(
            Image, '/krsbi/camera/omni/image_raw',
            self.omni_callback, image_qos)
        
        # =================================================================
        # Publishers
        # =================================================================
        if MSGS_AVAILABLE:
            self.ball_pub = self.create_publisher(
                BallPosition, '/krsbi/vision/ball', 10)
        
        self.point_pub = self.create_publisher(
            PointStamped, '/krsbi/vision/ball/point', 10)
        
        if self.publish_debug:
            self.debug_pub = self.create_publisher(
                Image, '/krsbi/vision/ball/debug', image_qos)
        
        # =================================================================
        # Timer
        # =================================================================
        period = 1.0 / self.detection_rate
        self.timer = self.create_timer(period, self.detect_ball)
        
        self.get_logger().info('Ball Detector started')
    
    def load_yolo(self):
        """Load YOLO model."""
        try:
            self.model = YOLO(self.yolo_model)
            self.get_logger().info(f'Loaded YOLO: {self.yolo_model}')
        except Exception as e:
            self.get_logger().error(f'Failed to load YOLO: {e}')
            self.model = None
            self.use_yolo = False
    
    def front_callback(self, msg: Image):
        """Handle front camera image."""
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            with self.lock:
                self.front_frame = frame
        except Exception as e:
            self.get_logger().error(f'Front image error: {e}')
    
    def omni_callback(self, msg: Image):
        """Handle omni camera image."""
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            with self.lock:
                self.omni_frame = frame
        except Exception as e:
            self.get_logger().error(f'Omni image error: {e}')
    
    def detect_ball(self):
        """Main detection loop."""
        with self.lock:
            front = self.front_frame.copy() if self.front_frame is not None else None
            omni = self.omni_frame.copy() if self.omni_frame is not None else None
        
        detections = []
        
        # Detect from front camera
        if front is not None:
            front_det = self.detect_in_frame(front, 'front')
            if front_det:
                detections.append(front_det)
        
        # Detect from omni camera
        if omni is not None:
            omni_det = self.detect_in_frame(omni, 'omni')
            if omni_det:
                detections.append(omni_det)
        
        # Strategy: Publish ALL detections, let the seeker node decide
        # But for now, let's prioritize closest or front
        
        best_detection = None
        if detections:
            # Prefer front camera if available (more accurate distance)
            front_d = next((d for d in detections if 'front' in d.source), None)
            if front_d:
                best_detection = front_d
            else:
                # Otherwise use omni
                best_detection = next((d for d in detections if 'omni' in d.source), None)
        
        self.last_detection = best_detection
        
        # Publish results
        self.publish_ball(best_detection)
        
        # Publish debug image (simple: just show what we have)
        if self.publish_debug:
            if best_detection and 'front' in best_detection.source and front is not None:
                self.publish_debug_image(front, best_detection)
            elif omni is not None:
                self.publish_debug_image(omni, best_detection)
            elif front is not None:
                self.publish_debug_image(front, None)
    
    def detect_in_frame(
        self, 
        frame: np.ndarray, 
        source: str
    ) -> Optional[BallDetection]:
        """
        Detect ball in single frame.
        
        Args:
            frame: Input BGR image
            source: Camera source ('front' or 'omni')
            
        Returns:
            Best detection or None
        """
        detections = []
        
        # YOLO detection
        if self.use_yolo and self.model:
            yolo_det = self.detect_yolo(frame, source)
            if yolo_det:
                detections.append(yolo_det)
        
        # Color-based detection
        if self.use_color:
            color_det = self.detect_color(frame, source)
            if color_det:
                detections.append(color_det)
        
        # Return best detection
        if not detections:
            return None
        
        return max(detections, key=lambda d: d.confidence)
    
    def detect_yolo(
        self, 
        frame: np.ndarray, 
        source: str
    ) -> Optional[BallDetection]:
        """Detect ball using YOLO."""
        try:
            # Select focal length
            focal_length = self.focal_length_omni if 'omni' in source else self.focal_length_front
            
            # Detect ball class (custom model: 0, COCO: 32)
            results = self.model.predict(
                frame,
                conf=self.yolo_conf,
                classes=[self.ball_class_id],  # Use configurable class ID
                verbose=False,
            )
            
            for result in results:
                boxes = result.boxes
                if boxes is None or len(boxes) == 0:
                    continue
                
                # Find best ball-like detection (check aspect ratio)
                best_detection = None
                best_conf = 0
                
                for i in range(len(boxes)):
                    xyxy = boxes.xyxy[i].cpu().numpy()
                    x1, y1, x2, y2 = xyxy
                    
                    width = x2 - x1
                    height = y2 - y1
                    
                    # Check aspect ratio (ball should be roughly square)
                    aspect_ratio = max(width, height) / (min(width, height) + 1e-6)
                    if aspect_ratio > 2.0:  # If too elongated, probably not a ball
                        continue
                        
                    conf = float(boxes.conf[i])
                    if conf > best_conf:
                        best_conf = conf
                        best_detection = {
                            'xyxy': xyxy,
                            'conf': conf
                        }
                
                if best_detection:
                    x1, y1, x2, y2 = best_detection['xyxy']
                    cx = (x1 + x2) / 2
                    cy = (y1 + y2) / 2
                    radius = max(x2 - x1, y2 - y1) / 2
                    
                    if 'omni' in source:
                        distance = -1.0
                    else:
                        distance = estimate_ball_distance(
                            radius, self.ball_diameter, focal_length
                        )
                    
                    # Debug log parameter for calibration
                    if self.publish_debug:
                        self.get_logger().debug(f"[{source}] Radius: {radius:.1f}px -> Dist: {distance:.2f}m (FL: {focal_length})")
                    
                    return BallDetection(
                        x=cx, y=cy, radius=radius,
                        confidence=best_detection['conf'],
                        source=f'yolo_{source}',
                        distance=distance,
                    )
            
            return None
            
        except Exception as e:
            self.get_logger().debug(f'YOLO error: {e}')
            return None
    
    def detect_color(
        self, 
        frame: np.ndarray, 
        source: str
    ) -> Optional[BallDetection]:
        """Detect ball using color segmentation."""
        # Select focal length
        focal_length = self.focal_length_omni if 'omni' in source else self.focal_length_front

        # Apply CLAHE for better color detection
        enhanced = apply_clahe(frame)
        
        # Create color mask - primary range (e.g., 140-180 for pink)
        mask1 = create_mask_hsv(
            enhanced,
            self.hue_low, self.hue_high,
            self.sat_low, self.sat_high,
            self.val_low, self.val_high,
            kernel_size=5,
        )
        
        # Create color mask - secondary range for wraparound (e.g., 0-15)
        mask2 = create_mask_hsv(
            enhanced,
            self.hue_low2, self.hue_high2,
            self.sat_low, self.sat_high,
            self.val_low, self.val_high,
            kernel_size=5,
        )
        
        # Combine both masks (for colors like pink that wrap around 180°)
        mask = cv2.bitwise_or(mask1, mask2)
        
        # Find contours
        contour = find_largest_contour(mask, self.min_area)
        
        if contour is None:
            return None
        
        area = cv2.contourArea(contour)
        if area > self.max_area:
            return None
        
        # Check circularity
        circularity = calculate_circularity(contour)
        if circularity < self.min_circularity:
            return None
        
        # Fit circle
        cx, cy, radius = fit_circle(contour)
        
        # Estimate distance
        if 'omni' in source:
            distance = -1.0
        else:
            distance = estimate_ball_distance(
                radius, self.ball_diameter, focal_length
            )
        
        # Debug log parameter for calibration
        if self.publish_debug:
             self.get_logger().debug(f"[{source}] Radius: {radius:.1f}px -> Dist: {distance:.2f}m (FL: {focal_length})")
        
        # Confidence based on circularity and area
        confidence = circularity * min(1.0, area / 1000)
        
        return BallDetection(
            x=cx, y=cy, radius=radius,
            confidence=confidence,
            source=f'color_{source}',
            distance=distance,
        )
    
    def fuse_detections(
        self, 
        detections: List[BallDetection]
    ) -> Optional[BallDetection]:
        """Fuse multiple detections."""
        if not detections:
            return None
        
        if len(detections) == 1:
            return detections[0]
        
        # Weighted average based on confidence
        total_weight = sum(d.confidence for d in detections)
        
        x = sum(d.x * d.confidence for d in detections) / total_weight
        y = sum(d.y * d.confidence for d in detections) / total_weight
        radius = sum(d.radius * d.confidence for d in detections) / total_weight
        distance = sum(d.distance * d.confidence for d in detections) / total_weight
        
        # Prefer front camera YOLO
        best = max(detections, key=lambda d: (
            1.0 if 'front' in d.source else 0.5,
            1.0 if 'yolo' in d.source else 0.5,
            d.confidence
        ))
        
        return BallDetection(
            x=x, y=y, radius=radius,
            confidence=min(1.0, total_weight),
            source='fused',
            distance=distance,
        )
    
    def publish_ball(self, detection: Optional[BallDetection]):
        """Publish ball position."""
        stamp = self.get_clock().now().to_msg()
        
        # Publish PointStamped (always works)
        point_msg = PointStamped()
        point_msg.header.stamp = stamp
        point_msg.header.frame_id = 'base_link'
        
        if detection:
            point_msg.point.x = detection.distance
            point_msg.point.y = 0.0  # TODO: calculate from bearing
            point_msg.point.z = 0.0
        else:
            point_msg.point.x = -1.0  # Invalid
        
        self.point_pub.publish(point_msg)
        
        # Publish BallPosition (if available)
        if MSGS_AVAILABLE and detection:
            msg = BallPosition()
            msg.header.stamp = stamp
            msg.header.frame_id = 'base_link'
            
            # Position in world coordinates (meters)
            msg.x = detection.distance  # Forward distance
            msg.y = 0.0  # TODO: calculate lateral from bearing
            msg.z = 0.0  # Assume ground level
            
            # Polar coordinates
            msg.distance = detection.distance
            msg.angle = 0.0  # TODO: calculate from pixel position
            
            # Status
            msg.confidence = detection.confidence
            msg.is_visible = True
            msg.is_moving = False  # TODO: calculate from velocity
            
            # Velocity (from Kalman)
            if self.last_velocity:
                msg.vx = self.last_velocity[0]
                msg.vy = self.last_velocity[1]
            else:
                msg.vx = 0.0
                msg.vy = 0.0
            
            # Camera source
            if 'omni' in detection.source:
                msg.camera_source = BallPosition.CAMERA_OMNI
            elif 'front' in detection.source:
                msg.camera_source = BallPosition.CAMERA_FRONT
            else:
                msg.camera_source = BallPosition.CAMERA_FUSED
            
            self.ball_pub.publish(msg)
    
    def publish_debug_image(
        self, 
        frame: np.ndarray, 
        detection: Optional[BallDetection]
    ):
        """Publish debug image with visualization."""
        output = frame.copy()
        
        # Draw Cartesian Grid (Center Crosshair)
        h, w = output.shape[:2]
        cx, cy = w // 2, h // 2
        # Color: Cyan (255, 255, 0) in BGR
        cv2.line(output, (0, cy), (w, cy), (255, 255, 0), 1)       # Horizontal X-axis
        cv2.line(output, (cx, 0), (cx, h), (255, 255, 0), 1)       # Vertical Y-axis
        cv2.circle(output, (cx, cy), 3, (0, 0, 255), -1)           # Center Point (Red)
        cv2.putText(output, "(0,0)", (cx + 5, cy - 5), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1)
        
        # Draw detection
        if detection:
            label = f"{detection.source}"
            if detection.distance > 0:
                label += f" {detection.distance:.2f}m"
            
            output = draw_detection(
                output, 
                (int(detection.x - detection.radius), int(detection.y - detection.radius),
                 int(detection.x + detection.radius), int(detection.y + detection.radius)),
                label,
                detection.confidence
            )
            
            # Draw center point and line to center
            cv2.line(output, (cx, cy), (int(detection.x), int(detection.y)), (0, 255, 255), 1)
            
            # Draw tracked position
            if self.last_position:
                cv2.circle(output, 
                          (int(self.last_position[0]), int(self.last_position[1])),
                          5, (0, 255, 0), -1)
            
            # Draw predicted trajectory
            if self.tracker:
                trajectory = self.tracker.predict_trajectory(duration=0.5)
                if trajectory:
                    output = draw_trajectory(output, trajectory)
        else:
            # Draw "NOT DETECTED" text
            cv2.putText(output, "Ball: NOT DETECTED", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        
        # Publish
        try:
            msg = self.bridge.cv2_to_imgmsg(output, encoding='bgr8')
            msg.header.stamp = self.get_clock().now().to_msg()
            self.debug_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f'Debug publish error: {e}')



def main(args=None):
    rclpy.init(args=args)
    
    node = BallDetectorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
