#!/usr/bin/env python3
"""
KRSBI-B Soccer Robot - Omni Camera Node

Captures and publishes images from omnidirectional fisheye camera.
Provides both raw fisheye and unwrapped panoramic images.

Publishes:
    - /krsbi/camera/omni/image_raw (sensor_msgs/Image) - Raw fisheye
    - /krsbi/camera/omni/image_unwrapped (sensor_msgs/Image) - Panoramic
    - /krsbi/camera/omni/camera_info (sensor_msgs/CameraInfo)
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
from typing import Optional, Tuple

from .utils import create_unwrap_maps, unwrap_fisheye_fast


class OmniCameraNode(Node):
    """
    Omnidirectional fisheye camera capture and unwrap node.
    """
    
    def __init__(self):
        super().__init__('omni_camera_node')
        
        # =================================================================
        # Parameters
        # =================================================================
        self.declare_parameter('device_id', 0)
        self.declare_parameter('frame_id', 'omni_camera_link')
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 30)
        self.declare_parameter('publish_rate', 30.0)
        
        # Fisheye parameters
        self.declare_parameter('mirror_center_x', 320)
        self.declare_parameter('mirror_center_y', 240)
        self.declare_parameter('inner_radius', 60)
        self.declare_parameter('outer_radius', 230)
        self.declare_parameter('unwrap_width', 640)
        self.declare_parameter('unwrap_height', 120)
        self.declare_parameter('angle_offset', 0.0)
        
        # Calibration
        self.declare_parameter('fx', 200.0)
        self.declare_parameter('fy', 200.0)
        self.declare_parameter('cx', 320.0)
        self.declare_parameter('cy', 240.0)
        
        self.device_id = self.get_parameter('device_id').value
        self.frame_id = self.get_parameter('frame_id').value
        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value
        self.fps = self.get_parameter('fps').value
        self.publish_rate = self.get_parameter('publish_rate').value
        
        # Fisheye config
        self.mirror_center = (
            self.get_parameter('mirror_center_x').value,
            self.get_parameter('mirror_center_y').value,
        )
        self.inner_radius = self.get_parameter('inner_radius').value
        self.outer_radius = self.get_parameter('outer_radius').value
        self.unwrap_width = self.get_parameter('unwrap_width').value
        self.unwrap_height = self.get_parameter('unwrap_height').value
        self.angle_offset = self.get_parameter('angle_offset').value
        
        # Calibration
        self.fx = self.get_parameter('fx').value
        self.fy = self.get_parameter('fy').value
        self.cx = self.get_parameter('cx').value
        self.cy = self.get_parameter('cy').value
        
        # =================================================================
        # Create unwrap maps
        # =================================================================
        self.map_x, self.map_y = create_unwrap_maps(
            image_size=(self.width, self.height),
            center=self.mirror_center,
            inner_radius=self.inner_radius,
            outer_radius=self.outer_radius,
            output_size=(self.unwrap_width, self.unwrap_height),
            angle_offset=self.angle_offset,
        )
        
        self.get_logger().info(
            f'Created unwrap maps: {self.unwrap_width}x{self.unwrap_height}'
        )
        
        # =================================================================
        # State
        # =================================================================
        self.bridge = CvBridge()
        self.cap: Optional[cv2.VideoCapture] = None
        self.is_running = True
        self.frame_count = 0
        self.last_frame: Optional[np.ndarray] = None
        self.last_unwrapped: Optional[np.ndarray] = None
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
        self.raw_pub = self.create_publisher(
            Image, '/krsbi/camera/omni/image_raw', image_qos)
        
        self.unwrapped_pub = self.create_publisher(
            Image, '/krsbi/camera/omni/image_unwrapped', image_qos)
        
        self.info_pub = self.create_publisher(
            CameraInfo, '/krsbi/camera/omni/camera_info', 10)
        
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
        
        self.get_logger().info(f'Omni Camera Node started (device: {self.device_id})')
    
    def init_camera(self):
        """Initialize camera capture."""
        self.cap = cv2.VideoCapture(self.device_id, cv2.CAP_V4L2)
        
        if not self.cap.isOpened():
            self.get_logger().warn(f'V4L2 failed, trying default backend')
            self.cap = cv2.VideoCapture(self.device_id)
        
        if self.cap.isOpened():
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            self.cap.set(cv2.CAP_PROP_FPS, self.fps)
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            
            actual_w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            actual_h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            actual_fps = self.cap.get(cv2.CAP_PROP_FPS)
            
            self.get_logger().info(
                f'Omni camera opened: {actual_w}x{actual_h} @ {actual_fps}fps'
            )
        else:
            self.get_logger().error('Failed to open omni camera')
    
    def capture_loop(self):
        """Background thread for camera capture."""
        while self.is_running:
            if self.cap is None or not self.cap.isOpened():
                time.sleep(0.1)
                continue
            
            ret, frame = self.cap.read()
            
            if ret:
                # Unwrap fisheye
                unwrapped = unwrap_fisheye_fast(frame, self.map_x, self.map_y)
                
                with self.lock:
                    self.last_frame = frame
                    self.last_unwrapped = unwrapped
                    self.frame_count += 1
            else:
                time.sleep(0.01)
    
    def publish_frame(self):
        """Publish current frames."""
        with self.lock:
            raw_frame = self.last_frame
            unwrapped_frame = self.last_unwrapped
        
        if raw_frame is None:
            return
        
        stamp = self.get_clock().now().to_msg()
        
        # Publish raw fisheye
        try:
            raw_msg = self.bridge.cv2_to_imgmsg(raw_frame, encoding='bgr8')
            raw_msg.header.stamp = stamp
            raw_msg.header.frame_id = self.frame_id
            self.raw_pub.publish(raw_msg)
        except Exception as e:
            self.get_logger().error(f'Failed to publish raw: {e}')
        
        # Publish unwrapped panorama
        if unwrapped_frame is not None:
            try:
                unwrap_msg = self.bridge.cv2_to_imgmsg(unwrapped_frame, encoding='bgr8')
                unwrap_msg.header.stamp = stamp
                unwrap_msg.header.frame_id = self.frame_id + '_unwrapped'
                self.unwrapped_pub.publish(unwrap_msg)
            except Exception as e:
                self.get_logger().error(f'Failed to publish unwrapped: {e}')
        
        # Publish camera info
        info_msg = self.create_camera_info(stamp)
        self.info_pub.publish(info_msg)
    
    def create_camera_info(self, stamp) -> CameraInfo:
        """Create CameraInfo message for fisheye camera."""
        msg = CameraInfo()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        
        msg.width = self.width
        msg.height = self.height
        
        # Fisheye model
        msg.distortion_model = 'equidistant'
        msg.d = [0.0, 0.0, 0.0, 0.0]  # Fisheye coefficients
        
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
    
    def get_bearing_from_unwrapped(self, x: int) -> float:
        """
        Convert x-coordinate in unwrapped image to bearing angle.
        
        Args:
            x: X-coordinate in unwrapped image
            
        Returns:
            Bearing angle in radians (-π to π, 0 = front)
        """
        # Unwrapped image spans 360 degrees
        angle = (x / self.unwrap_width) * 2 * np.pi + np.radians(self.angle_offset)
        
        # Normalize to [-π, π]
        while angle > np.pi:
            angle -= 2 * np.pi
        while angle < -np.pi:
            angle += 2 * np.pi
        
        return angle
    
    def get_distance_from_unwrapped(self, y: int, object_height: float) -> float:
        """
        Estimate distance from y-coordinate in unwrapped image.
        
        Args:
            y: Y-coordinate in unwrapped image
            object_height: Object height in meters
            
        Returns:
            Estimated distance in meters
        """
        # Map y to radius in original image
        radius = self.inner_radius + (y / self.unwrap_height) * (
            self.outer_radius - self.inner_radius
        )
        
        # Use focal length relationship
        # This is approximate for omni cameras
        if radius > 10:
            distance = (self.outer_radius / radius) * 2.0  # Rough estimate
        else:
            distance = float('inf')
        
        return distance
    
    def destroy_node(self):
        """Clean up."""
        self.is_running = False
        if self.cap:
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    node = OmniCameraNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
