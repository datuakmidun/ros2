#!/usr/bin/env python3
"""
KRSBI-B Soccer Robot - Robot Detector Node

Detects other robots using YOLO and multi-object tracking.

Subscribes:
    - /krsbi/camera/front/image_raw (sensor_msgs/Image)
    - /krsbi/camera/omni/image_unwrapped (sensor_msgs/Image)

Publishes:
    - /krsbi/vision/robots (krsbi_msgs/RobotsArray) [if available]
    - /krsbi/vision/robots/debug (sensor_msgs/Image)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseArray, Pose

import cv2
import numpy as np
from cv_bridge import CvBridge
import threading
from typing import Optional, List, Tuple
from dataclasses import dataclass

from .kalman_filter import MultiObjectTracker

# Try to import YOLO
try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False


@dataclass
class RobotDetection:
    """Robot detection result."""
    id: int
    x: float
    y: float
    width: float
    height: float
    confidence: float
    team: str  # 'unknown', 'blue', 'yellow'
    distance: float


class RobotDetectorNode(Node):
    """
    Robot/obstacle detector with multi-object tracking.
    """
    
    def __init__(self):
        super().__init__('robot_detector')
        
        # =================================================================
        # Parameters
        # =================================================================
        self.declare_parameter('use_yolo', True)
        self.declare_parameter('yolo_model', 'yolov8s.pt')
        self.declare_parameter('yolo_confidence', 0.5)
        
        # Team color detection (HSV)
        self.declare_parameter('blue_hue_low', 100)
        self.declare_parameter('blue_hue_high', 130)
        self.declare_parameter('yellow_hue_low', 20)
        self.declare_parameter('yellow_hue_high', 40)
        
        # Tracking
        self.declare_parameter('tracking_enabled', True)
        self.declare_parameter('max_age', 50)
        self.declare_parameter('min_hits', 5)
        
        # Debug
        self.declare_parameter('publish_debug', True)
        self.declare_parameter('detection_rate', 20.0)
        
        self.use_yolo = self.get_parameter('use_yolo').value
        self.yolo_model = self.get_parameter('yolo_model').value
        self.yolo_conf = self.get_parameter('yolo_confidence').value
        self.tracking_enabled = self.get_parameter('tracking_enabled').value
        self.publish_debug = self.get_parameter('publish_debug').value
        self.detection_rate = self.get_parameter('detection_rate').value
        
        # Team colors
        self.blue_hue = (
            self.get_parameter('blue_hue_low').value,
            self.get_parameter('blue_hue_high').value,
        )
        self.yellow_hue = (
            self.get_parameter('yellow_hue_low').value,
            self.get_parameter('yellow_hue_high').value,
        )
        
        # =================================================================
        # State
        # =================================================================
        self.bridge = CvBridge()
        self.model: Optional[YOLO] = None
        
        # Frame buffers
        self.front_frame: Optional[np.ndarray] = None
        self.omni_frame: Optional[np.ndarray] = None
        self.lock = threading.Lock()
        
        # Multi-object tracker
        if self.tracking_enabled:
            self.tracker = MultiObjectTracker(
                max_age=self.get_parameter('max_age').value,
                min_hits=self.get_parameter('min_hits').value,
            )
        else:
            self.tracker = None
        
        self.detected_robots: List[RobotDetection] = []
        
        # =================================================================
        # Load YOLO
        # =================================================================
        if self.use_yolo and YOLO_AVAILABLE:
            try:
                self.model = YOLO(self.yolo_model)
                self.get_logger().info(f'Loaded YOLO: {self.yolo_model}')
            except Exception as e:
                self.get_logger().error(f'Failed to load YOLO: {e}')
                self.use_yolo = False
        
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
        self.robots_pub = self.create_publisher(
            PoseArray, '/krsbi/vision/robots', 10)
        
        if self.publish_debug:
            self.debug_pub = self.create_publisher(
                Image, '/krsbi/vision/robots/debug', image_qos)
        
        # =================================================================
        # Timer
        # =================================================================
        period = 1.0 / self.detection_rate
        self.timer = self.create_timer(period, self.detect_robots)
        
        self.get_logger().info('Robot Detector started')
    
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
    
    def detect_robots(self):
        """Main detection loop."""
        with self.lock:
            front = self.front_frame.copy() if self.front_frame is not None else None
            omni = self.omni_frame.copy() if self.omni_frame is not None else None
        
        all_detections = []
        debug_frame = None
        
        # Detect from front camera
        if front is not None:
            dets = self.detect_in_frame(front)
            all_detections.extend(dets)
            debug_frame = front
        
        # Detect from omni camera
        if omni is not None:
            dets = self.detect_in_frame(omni)
            all_detections.extend(dets)
        
        # Update tracker
        if self.tracker and all_detections:
            positions = [np.array([d.x, d.y]) for d in all_detections]
            tracks = self.tracker.update(positions)
            
            # Update robot IDs from tracks
            self.detected_robots = []
            for track in tracks:
                pos = track.state[:2]
                robot = RobotDetection(
                    id=track.id,
                    x=pos[0], y=pos[1],
                    width=50, height=80,
                    confidence=track.confidence,
                    team='unknown',
                    distance=0.0,
                )
                self.detected_robots.append(robot)
        else:
            self.detected_robots = all_detections
        
        # Publish
        self.publish_robots()
        
        if self.publish_debug and debug_frame is not None:
            self.publish_debug_image(debug_frame)
    
    def detect_in_frame(self, frame: np.ndarray) -> List[RobotDetection]:
        """Detect robots in frame using YOLO."""
        detections = []
        
        if not self.use_yolo or self.model is None:
            return detections
        
        try:
            results = self.model.predict(
                frame,
                conf=self.yolo_conf,
                classes=[0],  # COCO person class
                verbose=False,
            )
            
            for result in results:
                boxes = result.boxes
                if boxes is None:
                    continue
                
                for i in range(len(boxes)):
                    xyxy = boxes.xyxy[i].cpu().numpy()
                    x1, y1, x2, y2 = [int(v) for v in xyxy]
                    conf = float(boxes.conf[i])
                    
                    cx = (x1 + x2) / 2
                    cy = (y1 + y2) / 2
                    w = x2 - x1
                    h = y2 - y1
                    
                    # Detect team color
                    roi = frame[y1:y2, x1:x2]
                    team = self.detect_team_color(roi)
                    
                    detections.append(RobotDetection(
                        id=0,
                        x=cx, y=cy,
                        width=w, height=h,
                        confidence=conf,
                        team=team,
                        distance=0.0,
                    ))
            
        except Exception as e:
            self.get_logger().debug(f'Detection error: {e}')
        
        return detections
    
    def detect_team_color(self, roi: np.ndarray) -> str:
        """Detect team color in ROI."""
        if roi.size == 0:
            return 'unknown'
        
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        
        # Blue mask
        blue_mask = cv2.inRange(hsv,
            np.array([self.blue_hue[0], 100, 80]),
            np.array([self.blue_hue[1], 255, 255]))
        blue_pixels = np.sum(blue_mask > 0)
        
        # Yellow mask
        yellow_mask = cv2.inRange(hsv,
            np.array([self.yellow_hue[0], 100, 100]),
            np.array([self.yellow_hue[1], 255, 255]))
        yellow_pixels = np.sum(yellow_mask > 0)
        
        total = roi.shape[0] * roi.shape[1]
        blue_ratio = blue_pixels / total
        yellow_ratio = yellow_pixels / total
        
        threshold = 0.05
        
        if blue_ratio > threshold and blue_ratio > yellow_ratio:
            return 'blue'
        elif yellow_ratio > threshold and yellow_ratio > blue_ratio:
            return 'yellow'
        else:
            return 'unknown'
    
    def publish_robots(self):
        """Publish detected robots."""
        msg = PoseArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        
        for robot in self.detected_robots:
            pose = Pose()
            pose.position.x = robot.x
            pose.position.y = robot.y
            pose.position.z = 0.0
            msg.poses.append(pose)
        
        self.robots_pub.publish(msg)
    
    def publish_debug_image(self, frame: np.ndarray):
        """Publish debug image."""
        output = frame.copy()
        
        colors = {
            'blue': (255, 0, 0),
            'yellow': (0, 255, 255),
            'unknown': (128, 128, 128),
        }
        
        for robot in self.detected_robots:
            color = colors.get(robot.team, (0, 255, 0))
            x, y = int(robot.x), int(robot.y)
            w, h = int(robot.width / 2), int(robot.height / 2)
            
            cv2.rectangle(output, (x - w, y - h), (x + w, y + h), color, 2)
            cv2.circle(output, (x, y), 4, (255, 255, 255), -1)
            
            label = f'R{robot.id} {robot.team}'
            cv2.putText(output, label, (x - w, y - h - 5),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        
        try:
            msg = self.bridge.cv2_to_imgmsg(output, encoding='bgr8')
            msg.header.stamp = self.get_clock().now().to_msg()
            self.debug_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f'Debug publish error: {e}')


def main(args=None):
    rclpy.init(args=args)
    
    node = RobotDetectorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
