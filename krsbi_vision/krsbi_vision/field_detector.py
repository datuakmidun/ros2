#!/usr/bin/env python3
"""
KRSBI-B Soccer Robot - Field Detector Node

Detects field boundaries, lines, and features.

Subscribes:
    - /krsbi/camera/front/image_raw (sensor_msgs/Image)
    - /krsbi/camera/omni/image_unwrapped (sensor_msgs/Image)

Publishes:
    - /krsbi/vision/field/mask (sensor_msgs/Image)
    - /krsbi/vision/field/lines (visualization_msgs/Marker)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

import cv2
import numpy as np
from cv_bridge import CvBridge
import threading
from typing import Optional, List, Tuple


class FieldDetectorNode(Node):
    """
    Field boundary and line detector.
    """
    
    def __init__(self):
        super().__init__('field_detector')
        
        # =================================================================
        # Parameters
        # =================================================================
        # Green field color (HSV)
        self.declare_parameter('field_hue_low', 35)
        self.declare_parameter('field_hue_high', 85)
        self.declare_parameter('field_sat_low', 50)
        self.declare_parameter('field_val_low', 50)
        
        # White lines (HSV)
        self.declare_parameter('line_sat_high', 50)
        self.declare_parameter('line_val_low', 200)
        
        # Line detection
        self.declare_parameter('hough_threshold', 50)
        self.declare_parameter('min_line_length', 50)
        self.declare_parameter('max_line_gap', 10)
        
        self.declare_parameter('publish_debug', True)
        self.declare_parameter('detection_rate', 10.0)
        
        # Get params
        self.field_hue = (
            self.get_parameter('field_hue_low').value,
            self.get_parameter('field_hue_high').value,
        )
        self.field_sat_low = self.get_parameter('field_sat_low').value
        self.field_val_low = self.get_parameter('field_val_low').value
        
        self.line_sat_high = self.get_parameter('line_sat_high').value
        self.line_val_low = self.get_parameter('line_val_low').value
        
        self.hough_threshold = self.get_parameter('hough_threshold').value
        self.min_line_length = self.get_parameter('min_line_length').value
        self.max_line_gap = self.get_parameter('max_line_gap').value
        
        self.publish_debug = self.get_parameter('publish_debug').value
        
        # =================================================================
        # State
        # =================================================================
        self.bridge = CvBridge()
        self.front_frame: Optional[np.ndarray] = None
        self.omni_frame: Optional[np.ndarray] = None
        self.lock = threading.Lock()
        
        self.detected_lines: List[Tuple[int, int, int, int]] = []
        self.field_mask: Optional[np.ndarray] = None
        
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
        self.mask_pub = self.create_publisher(
            Image, '/krsbi/vision/field/mask', image_qos)
        
        self.lines_pub = self.create_publisher(
            MarkerArray, '/krsbi/vision/field/lines', 10)
        
        if self.publish_debug:
            self.debug_pub = self.create_publisher(
                Image, '/krsbi/vision/field/debug', image_qos)
        
        # =================================================================
        # Timer
        # =================================================================
        rate = self.get_parameter('detection_rate').value
        self.timer = self.create_timer(1.0 / rate, self.detect_field)
        
        self.get_logger().info('Field Detector started')
    
    def front_callback(self, msg: Image):
        """Handle front camera image."""
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            with self.lock:
                self.front_frame = frame
        except Exception as e:
            self.get_logger().error(f'Image error: {e}')
    
    def omni_callback(self, msg: Image):
        """Handle omni camera image."""
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            with self.lock:
                self.omni_frame = frame
        except Exception as e:
            self.get_logger().error(f'Image error: {e}')
    
    def detect_field(self):
        """Main detection loop."""
        with self.lock:
            front = self.front_frame.copy() if self.front_frame is not None else None
        
        if front is None:
            return
        
        # Detect green field
        field_mask = self.detect_green_field(front)
        self.field_mask = field_mask
        
        # Detect white lines
        lines = self.detect_lines(front, field_mask)
        self.detected_lines = lines
        
        # Publish mask
        self.publish_mask(field_mask)
        
        # Publish lines
        self.publish_lines(lines)
        
        # Publish debug
        if self.publish_debug:
            self.publish_debug_image(front, field_mask, lines)
    
    def detect_green_field(self, frame: np.ndarray) -> np.ndarray:
        """Detect green field region."""
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        lower = np.array([self.field_hue[0], self.field_sat_low, self.field_val_low])
        upper = np.array([self.field_hue[1], 255, 255])
        
        mask = cv2.inRange(hsv, lower, upper)
        
        # Morphological operations
        kernel = np.ones((7, 7), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        
        return mask
    
    def detect_lines(
        self, 
        frame: np.ndarray, 
        field_mask: np.ndarray
    ) -> List[Tuple[int, int, int, int]]:
        """Detect white lines within field area."""
        # Convert to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # White color mask
        lower = np.array([0, 0, self.line_val_low])
        upper = np.array([180, self.line_sat_high, 255])
        white_mask = cv2.inRange(hsv, lower, upper)
        
        # Only within field
        line_mask = cv2.bitwise_and(white_mask, field_mask)
        
        # Edge detection
        edges = cv2.Canny(line_mask, 50, 150)
        
        # Hough lines
        lines = cv2.HoughLinesP(
            edges,
            rho=1,
            theta=np.pi / 180,
            threshold=self.hough_threshold,
            minLineLength=self.min_line_length,
            maxLineGap=self.max_line_gap,
        )
        
        result = []
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                result.append((x1, y1, x2, y2))
        
        return result
    
    def publish_mask(self, mask: np.ndarray):
        """Publish field mask."""
        try:
            msg = self.bridge.cv2_to_imgmsg(mask, encoding='mono8')
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'camera_link'
            self.mask_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f'Mask publish error: {e}')
    
    def publish_lines(self, lines: List[Tuple[int, int, int, int]]):
        """Publish detected lines as markers."""
        msg = MarkerArray()
        
        for i, (x1, y1, x2, y2) in enumerate(lines):
            marker = Marker()
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.header.frame_id = 'camera_link'
            marker.ns = 'field_lines'
            marker.id = i
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            
            # Convert to approximate 3D (assuming ground plane)
            p1 = Point(x=float(x1), y=float(y1), z=0.0)
            p2 = Point(x=float(x2), y=float(y2), z=0.0)
            marker.points = [p1, p2]
            
            marker.scale.x = 0.02
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 1.0
            marker.color.a = 1.0
            
            marker.lifetime.sec = 0
            marker.lifetime.nanosec = 100000000  # 100ms
            
            msg.markers.append(marker)
        
        self.lines_pub.publish(msg)
    
    def publish_debug_image(
        self, 
        frame: np.ndarray,
        mask: np.ndarray,
        lines: List[Tuple[int, int, int, int]]
    ):
        """Publish debug visualization."""
        output = frame.copy()
        
        # Overlay field mask
        green_overlay = np.zeros_like(frame)
        green_overlay[:, :, 1] = mask
        output = cv2.addWeighted(output, 0.7, green_overlay, 0.3, 0)
        
        # Draw lines
        for x1, y1, x2, y2 in lines:
            cv2.line(output, (x1, y1), (x2, y2), (0, 255, 255), 2)
        
        # Add info
        cv2.putText(output, f'Lines: {len(lines)}', (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        try:
            msg = self.bridge.cv2_to_imgmsg(output, encoding='bgr8')
            msg.header.stamp = self.get_clock().now().to_msg()
            self.debug_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f'Debug publish error: {e}')


def main(args=None):
    rclpy.init(args=args)
    
    node = FieldDetectorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
