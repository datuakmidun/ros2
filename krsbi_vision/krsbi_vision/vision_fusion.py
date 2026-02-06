#!/usr/bin/env python3
"""
KRSBI-B Soccer Robot - Vision Fusion Node

Fuses detections from both cameras (front + omni) into unified world coordinates.

Subscribes:
    - /krsbi/vision/ball (BallPosition)
    - /krsbi/vision/robots (PoseArray)
    - /krsbi/camera/front/camera_info (CameraInfo)
    - /krsbi/camera/omni/camera_info (CameraInfo)

Publishes:
    - /krsbi/vision/world/ball (PointStamped)
    - /krsbi/vision/world/robots (PoseArray)
    - /krsbi/vision/world/obstacles (OccupancyGrid)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, PoseArray, Pose, TransformStamped
from sensor_msgs.msg import CameraInfo
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header

import numpy as np
from typing import Optional, List, Dict, Tuple
from dataclasses import dataclass
import time
import math

try:
    from tf2_ros import TransformBroadcaster, Buffer, TransformListener
    TF2_AVAILABLE = True
except ImportError:
    TF2_AVAILABLE = False


@dataclass
class WorldObject:
    """Object in world coordinates."""
    id: int
    x: float              # meters, world frame
    y: float              # meters, world frame
    confidence: float
    last_seen: float      # timestamp
    object_type: str      # 'ball', 'robot', 'obstacle'


class VisionFusionNode(Node):
    """
    Fuses multi-camera detections into world coordinates.
    """
    
    def __init__(self):
        super().__init__('vision_fusion')
        
        # =================================================================
        # Parameters
        # =================================================================
        self.declare_parameter('robot_height', 0.35)
        self.declare_parameter('camera_height', 0.30)
        self.declare_parameter('front_camera_pitch', -0.26)  # -15 deg
        self.declare_parameter('max_detection_age', 0.5)
        self.declare_parameter('fusion_rate', 20.0)
        
        # Grid map parameters
        self.declare_parameter('map_size', 10.0)        # meters
        self.declare_parameter('map_resolution', 0.05)  # meters per cell
        
        self.robot_height = self.get_parameter('robot_height').value
        self.camera_height = self.get_parameter('camera_height').value
        self.camera_pitch = self.get_parameter('front_camera_pitch').value
        self.max_age = self.get_parameter('max_detection_age').value
        self.fusion_rate = self.get_parameter('fusion_rate').value
        
        self.map_size = self.get_parameter('map_size').value
        self.map_resolution = self.get_parameter('map_resolution').value
        
        # =================================================================
        # State
        # =================================================================
        self.world_ball: Optional[WorldObject] = None
        self.world_robots: Dict[int, WorldObject] = {}
        
        # Camera info
        self.front_camera_info: Optional[CameraInfo] = None
        self.omni_camera_info: Optional[CameraInfo] = None
        
        # Grid map
        self.grid_size = int(self.map_size / self.map_resolution)
        self.obstacle_grid = np.zeros((self.grid_size, self.grid_size), dtype=np.int8)
        
        # =================================================================
        # TF
        # =================================================================
        if TF2_AVAILABLE:
            self.tf_broadcaster = TransformBroadcaster(self)
        
        # =================================================================
        # Subscribers
        # =================================================================
        self.ball_sub = self.create_subscription(
            PointStamped, '/krsbi/vision/ball/tracked',
            self.ball_callback, 10)
        
        self.robots_sub = self.create_subscription(
            PoseArray, '/krsbi/vision/robots',
            self.robots_callback, 10)
        
        self.front_info_sub = self.create_subscription(
            CameraInfo, '/krsbi/camera/front/camera_info',
            self.front_info_callback, 10)
        
        self.omni_info_sub = self.create_subscription(
            CameraInfo, '/krsbi/camera/omni/camera_info',
            self.omni_info_callback, 10)
        
        # =================================================================
        # Publishers
        # =================================================================
        self.world_ball_pub = self.create_publisher(
            PointStamped, '/krsbi/vision/world/ball', 10)
        
        self.world_robots_pub = self.create_publisher(
            PoseArray, '/krsbi/vision/world/robots', 10)
        
        self.obstacle_grid_pub = self.create_publisher(
            OccupancyGrid, '/krsbi/vision/world/obstacles', 10)
        
        # =================================================================
        # Timer
        # =================================================================
        period = 1.0 / self.fusion_rate
        self.timer = self.create_timer(period, self.update)
        
        self.get_logger().info('Vision Fusion started')
    
    def front_info_callback(self, msg: CameraInfo):
        """Store front camera info."""
        self.front_camera_info = msg
    
    def omni_info_callback(self, msg: CameraInfo):
        """Store omni camera info."""
        self.omni_camera_info = msg
    
    def ball_callback(self, msg: PointStamped):
        """Handle ball detection."""
        if msg.point.x < 0:
            return  # Invalid detection
        
        # Transform pixel to world (simplified)
        world_x, world_y = self.pixel_to_world(
            msg.point.x, msg.point.y, 'front'
        )
        
        self.world_ball = WorldObject(
            id=0,
            x=world_x,
            y=world_y,
            confidence=1.0,
            last_seen=time.time(),
            object_type='ball',
        )
    
    def robots_callback(self, msg: PoseArray):
        """Handle robot detections."""
        current_time = time.time()
        
        for i, pose in enumerate(msg.poses):
            # Transform to world
            world_x, world_y = self.pixel_to_world(
                pose.position.x, pose.position.y, 'front'
            )
            
            robot_id = i + 1
            self.world_robots[robot_id] = WorldObject(
                id=robot_id,
                x=world_x,
                y=world_y,
                confidence=1.0,
                last_seen=current_time,
                object_type='robot',
            )
    
    def pixel_to_world(
        self, 
        u: float, 
        v: float, 
        camera: str
    ) -> Tuple[float, float]:
        """
        Convert pixel coordinates to world coordinates.
        
        Simplified projection assuming ground plane.
        """
        if camera == 'front' and self.front_camera_info:
            fx = self.front_camera_info.k[0]
            fy = self.front_camera_info.k[4]
            cx = self.front_camera_info.k[2]
            cy = self.front_camera_info.k[5]
        else:
            # Default values
            fx, fy = 554.0, 554.0
            cx, cy = 320.0, 240.0
        
        # Normalize
        x_norm = (u - cx) / fx
        y_norm = (v - cy) / fy
        
        # Ray direction with pitch
        cos_p = math.cos(self.camera_pitch)
        sin_p = math.sin(self.camera_pitch)
        
        dir_x = x_norm
        dir_y = y_norm * cos_p + sin_p
        dir_z = -y_norm * sin_p + cos_p
        
        if abs(dir_z) < 0.001:
            return 0.0, 0.0
        
        # Intersect with ground
        t = -self.camera_height / dir_z
        
        if t < 0:
            return 0.0, 0.0
        
        world_x = t * dir_x
        world_y = t * dir_y
        
        return world_x, world_y
    
    def update(self):
        """Periodic update and publishing."""
        current_time = time.time()
        
        # Clean up old detections
        if self.world_ball and (current_time - self.world_ball.last_seen) > self.max_age:
            self.world_ball = None
        
        old_robots = [
            rid for rid, robot in self.world_robots.items()
            if (current_time - robot.last_seen) > self.max_age
        ]
        for rid in old_robots:
            del self.world_robots[rid]
        
        # Publish
        self.publish_world_ball()
        self.publish_world_robots()
        self.update_and_publish_grid()
        
        # Broadcast TF
        if TF2_AVAILABLE and self.world_ball:
            self.broadcast_ball_tf()
    
    def publish_world_ball(self):
        """Publish ball in world coordinates."""
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        
        if self.world_ball:
            msg.point.x = self.world_ball.x
            msg.point.y = self.world_ball.y
            msg.point.z = 0.0
        else:
            msg.point.x = -1.0  # Invalid
        
        self.world_ball_pub.publish(msg)
    
    def publish_world_robots(self):
        """Publish robots in world coordinates."""
        msg = PoseArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        
        for robot in self.world_robots.values():
            pose = Pose()
            pose.position.x = robot.x
            pose.position.y = robot.y
            pose.position.z = 0.0
            msg.poses.append(pose)
        
        self.world_robots_pub.publish(msg)
    
    def update_and_publish_grid(self):
        """Update obstacle grid and publish."""
        # Fade existing obstacles
        self.obstacle_grid = (self.obstacle_grid * 0.95).astype(np.int8)
        
        # Add current robots to grid
        for robot in self.world_robots.values():
            # Convert world to grid coordinates
            gx = int((robot.x + self.map_size / 2) / self.map_resolution)
            gy = int((robot.y + self.map_size / 2) / self.map_resolution)
            
            if 0 <= gx < self.grid_size and 0 <= gy < self.grid_size:
                # Mark robot area as occupied
                for dx in range(-2, 3):
                    for dy in range(-2, 3):
                        nx, ny = gx + dx, gy + dy
                        if 0 <= nx < self.grid_size and 0 <= ny < self.grid_size:
                            self.obstacle_grid[ny, nx] = 100
        
        # Publish grid
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        
        msg.info.resolution = self.map_resolution
        msg.info.width = self.grid_size
        msg.info.height = self.grid_size
        msg.info.origin.position.x = -self.map_size / 2
        msg.info.origin.position.y = -self.map_size / 2
        msg.info.origin.position.z = 0.0
        
        msg.data = self.obstacle_grid.flatten().tolist()
        
        self.obstacle_grid_pub.publish(msg)
    
    def broadcast_ball_tf(self):
        """Broadcast ball position as TF."""
        if not self.world_ball or not TF2_AVAILABLE:
            return
        
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'ball'
        
        t.transform.translation.x = self.world_ball.x
        t.transform.translation.y = self.world_ball.y
        t.transform.translation.z = 0.0
        
        t.transform.rotation.w = 1.0
        
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    
    node = VisionFusionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
