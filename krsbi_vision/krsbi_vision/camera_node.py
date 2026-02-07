#!/usr/bin/env python3
"""
KRSBI-B Soccer Robot - Front Camera Node

Captures and publishes images from front-facing camera.

Publishes:
    - /krsbi/camera/front/image_raw (sensor_msgs/Image)
    - /krsbi/camera/front/camera_info (sensor_msgs/CameraInfo)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Header

import cv2
import numpy as np
from cv_bridge import CvBridge
import threading
import time
from typing import Optional


class CameraNode(Node):
    """
    Front camera capture and publish node.
    """
    
    def __init__(self):
        super().__init__('camera_node')
        
        # =================================================================
        # Parameters
        # =================================================================
        self.declare_parameter('device_id', 4)
        self.declare_parameter('frame_id', 'front_camera_link')
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 30)
        self.declare_parameter('auto_exposure', True)
        self.declare_parameter('publish_rate', 30.0)
        
        # Calibration parameters
        self.declare_parameter('fx', 554.25)
        self.declare_parameter('fy', 554.25)
        self.declare_parameter('cx', 320.0)
        self.declare_parameter('cy', 240.0)
        self.declare_parameter('k1', -0.1)
        self.declare_parameter('k2', 0.05)
        self.declare_parameter('p1', 0.0)
        self.declare_parameter('p2', 0.0)
        self.declare_parameter('k3', 0.0)
        
        self.device_id = self.get_parameter('device_id').value
        self.frame_id = self.get_parameter('frame_id').value
        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value
        self.fps = self.get_parameter('fps').value
        self.publish_rate = self.get_parameter('publish_rate').value
        
        # =================================================================
        # Camera calibration
        # =================================================================
        self.fx = self.get_parameter('fx').value
        self.fy = self.get_parameter('fy').value
        self.cx = self.get_parameter('cx').value
        self.cy = self.get_parameter('cy').value
        
        self.camera_matrix = np.array([
            [self.fx, 0, self.cx],
            [0, self.fy, self.cy],
            [0, 0, 1]
        ], dtype=np.float64)
        
        self.dist_coeffs = np.array([
            self.get_parameter('k1').value,
            self.get_parameter('k2').value,
            self.get_parameter('p1').value,
            self.get_parameter('p2').value,
            self.get_parameter('k3').value,
        ], dtype=np.float64)
        
        # =================================================================
        # State
        # =================================================================
        self.bridge = CvBridge()
        self.cap: Optional[cv2.VideoCapture] = None
        self.is_running = True
        self.frame_count = 0
        self.last_frame: Optional[np.ndarray] = None
        self.lock = threading.Lock()
        
        # =================================================================
        # QoS
        # =================================================================
        image_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # =================================================================
        # Publishers
        # =================================================================
        self.image_pub = self.create_publisher(
            Image, '/krsbi/camera/front/image_raw', image_qos)
        
        self.info_pub = self.create_publisher(
            CameraInfo, '/krsbi/camera/front/camera_info', 10)
        
        # =================================================================
        # Initialize camera
        # =================================================================
        self.init_camera()
        
        # =================================================================
        # Capture thread
        # =================================================================
        self.capture_thread = threading.Thread(target=self.capture_loop, daemon=True)
        self.capture_thread.start()
        
        # =================================================================
        # Publish timer
        # =================================================================
        period = 1.0 / self.publish_rate
        self.timer = self.create_timer(period, self.publish_frame)
        
        self.get_logger().info(f'Front Camera Node started (device: {self.device_id})')
    
    def init_camera(self):
        """Initialize camera capture."""
        self.cap = cv2.VideoCapture(self.device_id, cv2.CAP_V4L2)
        
        if not self.cap.isOpened():
            self.get_logger().error(f'Failed to open camera {self.device_id}')
            # Try without V4L2
            self.cap = cv2.VideoCapture(self.device_id)
        
        if self.cap.isOpened():
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            self.cap.set(cv2.CAP_PROP_FPS, self.fps)
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            
            # Read actual settings
            actual_w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            actual_h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            actual_fps = self.cap.get(cv2.CAP_PROP_FPS)
            
            self.get_logger().info(
                f'Camera opened: {actual_w}x{actual_h} @ {actual_fps}fps')
        else:
            self.get_logger().error('Failed to open camera')
    
    def capture_loop(self):
        """Background thread for camera capture."""
        while self.is_running:
            if self.cap is None or not self.cap.isOpened():
                time.sleep(0.1)
                continue
            
            ret, frame = self.cap.read()
            
            if ret:
                with self.lock:
                    self.last_frame = frame
                    self.frame_count += 1
            else:
                self.get_logger().warn('Failed to capture frame')
                time.sleep(0.01)
    
    def publish_frame(self):
        """Publish current frame."""
        with self.lock:
            frame = self.last_frame
        
        if frame is None:
            return
        
        # Create timestamp
        stamp = self.get_clock().now().to_msg()
        
        # Publish image
        try:
            img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            img_msg.header.stamp = stamp
            img_msg.header.frame_id = self.frame_id
            self.image_pub.publish(img_msg)
        except Exception as e:
            self.get_logger().error(f'Failed to publish image: {e}')
        
        # Publish camera info
        info_msg = self.create_camera_info(stamp)
        self.info_pub.publish(info_msg)
    
    def create_camera_info(self, stamp) -> CameraInfo:
        """Create CameraInfo message."""
        msg = CameraInfo()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        
        msg.width = self.width
        msg.height = self.height
        
        msg.distortion_model = 'plumb_bob'
        msg.d = self.dist_coeffs.tolist()
        
        msg.k = [
            self.fx, 0.0, self.cx,
            0.0, self.fy, self.cy,
            0.0, 0.0, 1.0
        ]
        
        msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        
        msg.p = [
            self.fx, 0.0, self.cx, 0.0,
            0.0, self.fy, self.cy, 0.0,
            0.0, 0.0, 1.0, 0.0
        ]
        
        return msg
    
    def destroy_node(self):
        """Clean up."""
        self.is_running = False
        if self.cap:
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    node = CameraNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
