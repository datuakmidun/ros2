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
    draw_ball, draw_trajectory, apply_clahe,
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
        self.declare_parameter('yolo_confidence', 0.6)
        
        # Color detection (HSV)
        self.declare_parameter('hue_low', 5)
        self.declare_parameter('hue_high', 25)
        self.declare_parameter('saturation_low', 100)
        self.declare_parameter('saturation_high', 255)
        self.declare_parameter('value_low', 100)
        self.declare_parameter('value_high', 255)
        
        # Size filtering
        self.declare_parameter('min_area', 100)
        self.declare_parameter('max_area', 50000)
        self.declare_parameter('min_circularity', 0.6)
        
        # Distance estimation
        self.declare_parameter('ball_diameter', 0.21)
        self.declare_parameter('focal_length', 554.0)
        
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
        
        self.hue_low = self.get_parameter('hue_low').value
        self.hue_high = self.get_parameter('hue_high').value
        self.sat_low = self.get_parameter('saturation_low').value
        self.sat_high = self.get_parameter('saturation_high').value
        self.val_low = self.get_parameter('value_low').value
        self.val_high = self.get_parameter('value_high').value
        
        self.min_area = self.get_parameter('min_area').value
        self.max_area = self.get_parameter('max_area').value
        self.min_circularity = self.get_parameter('min_circularity').value
        
        self.ball_diameter = self.get_parameter('ball_diameter').value
        self.focal_length = self.get_parameter('focal_length').value
        
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
            Image, '/krsbi/camera/omni/image_unwrapped',
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
        debug_frame = None
        
        # Detect from front camera
        if front is not None:
            front_det = self.detect_in_frame(front, 'front')
            if front_det:
                detections.append(front_det)
            debug_frame = front
        
        # Detect from omni camera
        if omni is not None:
            omni_det = self.detect_in_frame(omni, 'omni')
            if omni_det:
                detections.append(omni_det)
        
        # Fuse detections (prefer front camera, higher confidence)
        best_detection = self.fuse_detections(detections)
        
        # Update Kalman filter
        if self.tracker:
            if best_detection:
                measurement = np.array([best_detection.x, best_detection.y])
                track = self.tracker.update(measurement)
            else:
                track = self.tracker.update(None)
            
            # Get filtered position
            if track and track.is_confirmed:
                pos = self.tracker.get_position()
                vel = self.tracker.get_velocity()
                if pos is not None:
                    self.last_position = tuple(pos)
                if vel is not None:
                    self.last_velocity = tuple(vel)
        
        self.last_detection = best_detection
        
        # Publish results
        self.publish_ball(best_detection)
        
        # Publish debug image
        if self.publish_debug and debug_frame is not None:
            self.publish_debug_image(debug_frame, best_detection)
    
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
            results = self.model.predict(
                frame,
                conf=self.yolo_conf,
                classes=[32],  # COCO sports ball, or custom class
                verbose=False,
            )
            
            for result in results:
                boxes = result.boxes
                if boxes is None or len(boxes) == 0:
                    continue
                
                # Get best ball detection
                best_idx = 0
                best_conf = 0
                
                for i in range(len(boxes)):
                    conf = float(boxes.conf[i])
                    if conf > best_conf:
                        best_conf = conf
                        best_idx = i
                
                xyxy = boxes.xyxy[best_idx].cpu().numpy()
                x1, y1, x2, y2 = xyxy
                
                cx = (x1 + x2) / 2
                cy = (y1 + y2) / 2
                radius = max(x2 - x1, y2 - y1) / 2
                
                distance = estimate_ball_distance(
                    radius, self.ball_diameter, self.focal_length
                )
                
                return BallDetection(
                    x=cx, y=cy, radius=radius,
                    confidence=best_conf,
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
        # Apply CLAHE for better color detection
        enhanced = apply_clahe(frame)
        
        # Create color mask
        mask = create_mask_hsv(
            enhanced,
            self.hue_low, self.hue_high,
            self.sat_low, self.sat_high,
            self.val_low, self.val_high,
            kernel_size=5,
        )
        
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
        distance = estimate_ball_distance(
            radius, self.ball_diameter, self.focal_length
        )
        
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
        
        # Draw detection
        if detection:
            output = draw_ball(
                output,
                (detection.x, detection.y),
                detection.radius,
                distance=detection.distance,
            )
            
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
