#!/usr/bin/env python3
"""
KRSBI-B Soccer Robot - YOLO Detector Node

YOLOv8 object detection for ball, robots, and goals.

Subscribes:
    - /krsbi/camera/front/image_raw (sensor_msgs/Image)
    - /krsbi/camera/omni/image_unwrapped (sensor_msgs/Image)

Publishes:
    - /krsbi/yolo/detections (krsbi_msgs/DetectionArray) [if available]
    - /krsbi/yolo/debug (sensor_msgs/Image)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Header

import cv2
import numpy as np
from cv_bridge import CvBridge
import threading
import time
from typing import Optional, List, Tuple
from dataclasses import dataclass

# Try to import YOLO
try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False
    print("Warning: ultralytics not installed. Run: pip install ultralytics")

# Try to import krsbi_msgs
try:
    from krsbi_msgs.msg import Detection, DetectionArray
    MSGS_AVAILABLE = True
except ImportError:
    MSGS_AVAILABLE = False


@dataclass
class YoloDetection:
    """YOLO detection result."""
    class_id: int
    class_name: str
    confidence: float
    bbox: Tuple[int, int, int, int]  # x1, y1, x2, y2
    center: Tuple[float, float]
    area: float


class YoloDetectorNode(Node):
    """
    YOLOv8 object detection node.
    
    Supports:
    - Multiple camera inputs
    - Custom trained models
    - Real-time inference
    """
    
    def __init__(self):
        super().__init__('yolo_detector')
        
        # =================================================================
        # Parameters
        # =================================================================
        self.declare_parameter('model_path', 'yolov8s.pt')
        self.declare_parameter('custom_model', '')
        self.declare_parameter('device', 'cpu')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('iou_threshold', 0.45)
        self.declare_parameter('input_size', 416)
        self.declare_parameter('classes', [-1])  # -1 = all
        self.declare_parameter('publish_debug', True)
        self.declare_parameter('inference_rate', 20.0)
        
        self.model_path = self.get_parameter('model_path').value
        custom_model = self.get_parameter('custom_model').value
        if custom_model:
            self.model_path = custom_model
        
        self.device = self.get_parameter('device').value
        self.conf_thresh = self.get_parameter('confidence_threshold').value
        self.iou_thresh = self.get_parameter('iou_threshold').value
        self.input_size = self.get_parameter('input_size').value
        self.classes = self.get_parameter('classes').value
        self.publish_debug = self.get_parameter('publish_debug').value
        self.inference_rate = self.get_parameter('inference_rate').value
        
        # =================================================================
        # State
        # =================================================================
        self.bridge = CvBridge()
        self.model: Optional[YOLO] = None
        self.is_running = True
        
        # Frame buffers
        self.front_frame: Optional[np.ndarray] = None
        self.omni_frame: Optional[np.ndarray] = None
        self.front_stamp = None
        self.omni_stamp = None
        self.lock = threading.Lock()
        
        # Detection results
        self.last_detections: List[YoloDetection] = []
        
        # Class names (COCO default, can be overridden)
        self.class_names = {
            0: 'ball',
            1: 'robot',
            2: 'goalpost',
            # COCO classes (if using pretrained)
            32: 'ball',  # sports ball
            0: 'person',  # For robot proxy
        }
        
        # =================================================================
        # Load model
        # =================================================================
        self.load_model()
        
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
            self.detection_pub = self.create_publisher(
                DetectionArray, '/krsbi/yolo/detections', 10)
        
        if self.publish_debug:
            self.debug_front_pub = self.create_publisher(
                Image, '/krsbi/yolo/debug/front', image_qos)
            self.debug_omni_pub = self.create_publisher(
                Image, '/krsbi/yolo/debug/omni', image_qos)
        
        # =================================================================
        # Inference timer
        # =================================================================
        period = 1.0 / self.inference_rate
        self.timer = self.create_timer(period, self.run_inference)
        
        self.get_logger().info(f'YOLO Detector started (model: {self.model_path})')
    
    def load_model(self):
        """Load YOLO model."""
        if not YOLO_AVAILABLE:
            self.get_logger().error('YOLO not available')
            return
        
        try:
            self.model = YOLO(self.model_path)
            
            # Move to device
            if self.device.startswith('cuda'):
                self.model.to(self.device)
            
            self.get_logger().info(f'Loaded YOLO model: {self.model_path}')
            
            # Warmup
            dummy = np.zeros((self.input_size, self.input_size, 3), dtype=np.uint8)
            _ = self.model.predict(dummy, verbose=False)
            self.get_logger().info('Model warmup complete')
            
        except Exception as e:
            self.get_logger().error(f'Failed to load model: {e}')
            self.model = None
    
    def front_callback(self, msg: Image):
        """Handle front camera image."""
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            with self.lock:
                self.front_frame = frame
                self.front_stamp = msg.header.stamp
        except Exception as e:
            self.get_logger().error(f'Front image error: {e}')
    
    def omni_callback(self, msg: Image):
        """Handle omni camera image."""
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            with self.lock:
                self.omni_frame = frame
                self.omni_stamp = msg.header.stamp
        except Exception as e:
            self.get_logger().error(f'Omni image error: {e}')
    
    def run_inference(self):
        """Run YOLO inference on latest frames."""
        if self.model is None:
            return
        
        with self.lock:
            front = self.front_frame.copy() if self.front_frame is not None else None
            omni = self.omni_frame.copy() if self.omni_frame is not None else None
        
        all_detections = []
        
        # Process front camera
        if front is not None:
            detections = self.detect(front)
            all_detections.extend(detections)
            
            if self.publish_debug:
                debug_img = self.draw_detections(front, detections)
                self.publish_debug_image(debug_img, self.debug_front_pub)
        
        # Process omni camera
        if omni is not None:
            detections = self.detect(omni)
            all_detections.extend(detections)
            
            if self.publish_debug:
                debug_img = self.draw_detections(omni, detections)
                self.publish_debug_image(debug_img, self.debug_omni_pub)
        
        # Store and publish detections
        self.last_detections = all_detections
        
        if MSGS_AVAILABLE and all_detections:
            self.publish_detections(all_detections)
    
    def detect(self, image: np.ndarray) -> List[YoloDetection]:
        """
        Run YOLO detection on image.
        
        Args:
            image: BGR image
            
        Returns:
            List of detections
        """
        if self.model is None:
            return []
        
        try:
            # Run inference
            results = self.model.predict(
                image,
                conf=self.conf_thresh,
                iou=self.iou_thresh,
                imgsz=self.input_size,
                classes=self.classes if self.classes != [-1] else None,
                verbose=False,
            )
            
            detections = []
            
            for result in results:
                boxes = result.boxes
                
                if boxes is None:
                    continue
                
                for i in range(len(boxes)):
                    # Get box coordinates
                    xyxy = boxes.xyxy[i].cpu().numpy()
                    x1, y1, x2, y2 = [int(v) for v in xyxy]
                    
                    # Get class and confidence
                    conf = float(boxes.conf[i])
                    cls_id = int(boxes.cls[i])
                    cls_name = self.class_names.get(cls_id, f'class_{cls_id}')
                    
                    # Calculate center and area
                    center = ((x1 + x2) / 2, (y1 + y2) / 2)
                    area = (x2 - x1) * (y2 - y1)
                    
                    detections.append(YoloDetection(
                        class_id=cls_id,
                        class_name=cls_name,
                        confidence=conf,
                        bbox=(x1, y1, x2, y2),
                        center=center,
                        area=area,
                    ))
            
            return detections
            
        except Exception as e:
            self.get_logger().error(f'Detection error: {e}')
            return []
    
    def draw_detections(
        self, 
        image: np.ndarray, 
        detections: List[YoloDetection]
    ) -> np.ndarray:
        """Draw detections on image."""
        output = image.copy()
        
        colors = {
            'ball': (0, 165, 255),      # Orange
            'robot': (255, 0, 0),        # Blue
            'goalpost': (0, 255, 255),   # Yellow
        }
        
        for det in detections:
            color = colors.get(det.class_name, (0, 255, 0))
            x1, y1, x2, y2 = det.bbox
            
            # Draw box
            cv2.rectangle(output, (x1, y1), (x2, y2), color, 2)
            
            # Draw label
            label = f'{det.class_name}: {det.confidence:.2f}'
            (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            cv2.rectangle(output, (x1, y1 - th - 4), (x1 + tw, y1), color, -1)
            cv2.putText(output, label, (x1, y1 - 2),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
            
            # Draw center
            cx, cy = int(det.center[0]), int(det.center[1])
            cv2.circle(output, (cx, cy), 4, (255, 255, 255), -1)
        
        return output
    
    def publish_detections(self, detections: List[YoloDetection]):
        """Publish detection array."""
        if not MSGS_AVAILABLE:
            return
        
        msg = DetectionArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_link'
        
        for det in detections:
            d = Detection()
            d.class_id = det.class_id
            d.class_name = det.class_name
            d.confidence = det.confidence
            d.bbox_x = det.bbox[0]
            d.bbox_y = det.bbox[1]
            d.bbox_width = det.bbox[2] - det.bbox[0]
            d.bbox_height = det.bbox[3] - det.bbox[1]
            d.center_x = det.center[0]
            d.center_y = det.center[1]
            msg.detections.append(d)
        
        self.detection_pub.publish(msg)
    
    def publish_debug_image(self, image: np.ndarray, publisher):
        """Publish debug image."""
        try:
            msg = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
            msg.header.stamp = self.get_clock().now().to_msg()
            publisher.publish(msg)
        except Exception as e:
            self.get_logger().error(f'Debug publish error: {e}')
    
    def get_ball_detections(self) -> List[YoloDetection]:
        """Get ball detections only."""
        return [d for d in self.last_detections if d.class_name == 'ball']
    
    def get_robot_detections(self) -> List[YoloDetection]:
        """Get robot detections only."""
        return [d for d in self.last_detections if d.class_name == 'robot']


def main(args=None):
    rclpy.init(args=args)
    
    node = YoloDetectorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
