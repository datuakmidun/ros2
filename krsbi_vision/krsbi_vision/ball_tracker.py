#!/usr/bin/env python3
"""
KRSBI-B Soccer Robot - Ball Tracker Node

Standalone Kalman filter ball tracking node.
Subscribes to ball detections and publishes tracked/predicted positions.

Subscribes:
    - /krsbi/vision/ball/point (geometry_msgs/PointStamped)

Publishes:
    - /krsbi/vision/ball/tracked (geometry_msgs/PointStamped)
    - /krsbi/vision/ball/predicted (geometry_msgs/PoseArray)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, PoseArray, Pose
from std_msgs.msg import Float32

import numpy as np
import time
from typing import Optional, List

from .kalman_filter import BallTracker


class BallTrackerNode(Node):
    """
    Ball tracking node with Kalman filter.
    
    Features:
    - Constant acceleration model
    - Physics-based prediction (friction)
    - Trajectory prediction
    - Velocity estimation
    """
    
    def __init__(self):
        super().__init__('ball_tracker')
        
        # =================================================================
        # Parameters
        # =================================================================
        self.declare_parameter('dt', 0.02)
        self.declare_parameter('process_noise_pos', 0.1)
        self.declare_parameter('process_noise_vel', 1.0)
        self.declare_parameter('process_noise_acc', 5.0)
        self.declare_parameter('measurement_noise', 2.0)
        self.declare_parameter('max_age', 30)
        self.declare_parameter('min_hits', 3)
        self.declare_parameter('friction', 0.3)
        self.declare_parameter('max_velocity', 3.0)
        self.declare_parameter('prediction_duration', 1.0)
        self.declare_parameter('update_rate', 50.0)
        
        dt = self.get_parameter('dt').value
        process_noise_pos = self.get_parameter('process_noise_pos').value
        process_noise_vel = self.get_parameter('process_noise_vel').value
        process_noise_acc = self.get_parameter('process_noise_acc').value
        measurement_noise = self.get_parameter('measurement_noise').value
        max_age = self.get_parameter('max_age').value
        min_hits = self.get_parameter('min_hits').value
        friction = self.get_parameter('friction').value
        max_velocity = self.get_parameter('max_velocity').value
        
        self.prediction_duration = self.get_parameter('prediction_duration').value
        self.update_rate = self.get_parameter('update_rate').value
        
        # =================================================================
        # Initialize tracker
        # =================================================================
        self.tracker = BallTracker(
            dt=dt,
            process_noise_pos=process_noise_pos,
            process_noise_vel=process_noise_vel,
            process_noise_acc=process_noise_acc,
            measurement_noise=measurement_noise,
            max_age=max_age,
            min_hits=min_hits,
            friction=friction,
            max_velocity=max_velocity,
        )
        
        self.last_detection_time = time.time()
        
        # =================================================================
        # Subscribers
        # =================================================================
        self.detection_sub = self.create_subscription(
            PointStamped, '/krsbi/vision/ball/point',
            self.detection_callback, 10)
        
        # =================================================================
        # Publishers
        # =================================================================
        self.tracked_pub = self.create_publisher(
            PointStamped, '/krsbi/vision/ball/tracked', 10)
        
        self.predicted_pub = self.create_publisher(
            PoseArray, '/krsbi/vision/ball/predicted', 10)
        
        self.speed_pub = self.create_publisher(
            Float32, '/krsbi/vision/ball/speed', 10)
        
        # =================================================================
        # Timer
        # =================================================================
        period = 1.0 / self.update_rate
        self.timer = self.create_timer(period, self.update)
        
        self.get_logger().info('Ball Tracker started')
    
    def detection_callback(self, msg: PointStamped):
        """Handle ball detection."""
        # Check if valid detection
        if msg.point.x < 0:
            # Invalid detection
            return
        
        # Update with detection
        measurement = np.array([msg.point.x, msg.point.y])
        self.tracker.update(measurement)
        self.last_detection_time = time.time()
    
    def update(self):
        """Periodic update and publish."""
        # Check if we should predict without measurement
        time_since_detection = time.time() - self.last_detection_time
        if time_since_detection > 0.1:
            # No recent detection, just predict
            self.tracker.update(None)
        
        # Publish tracked position
        self.publish_tracked()
        
        # Publish predicted trajectory
        self.publish_predicted()
        
        # Publish speed
        self.publish_speed()
    
    def publish_tracked(self):
        """Publish current tracked position."""
        pos = self.tracker.get_position()
        
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        
        if pos is not None and self.tracker.is_tracking():
            msg.point.x = float(pos[0])
            msg.point.y = float(pos[1])
            msg.point.z = 0.0
        else:
            msg.point.x = -1.0  # Invalid
            msg.point.y = -1.0
            msg.point.z = -1.0
        
        self.tracked_pub.publish(msg)
    
    def publish_predicted(self):
        """Publish predicted trajectory."""
        if not self.tracker.is_tracking():
            return
        
        trajectory = self.tracker.predict_trajectory(
            duration=self.prediction_duration,
            step=0.05,
        )
        
        if not trajectory:
            return
        
        msg = PoseArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        
        for point in trajectory:
            pose = Pose()
            pose.position.x = float(point[0])
            pose.position.y = float(point[1])
            pose.position.z = 0.0
            msg.poses.append(pose)
        
        self.predicted_pub.publish(msg)
    
    def publish_speed(self):
        """Publish ball speed."""
        vel = self.tracker.get_velocity()
        
        msg = Float32()
        if vel is not None:
            msg.data = float(np.linalg.norm(vel))
        else:
            msg.data = 0.0
        
        self.speed_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    
    node = BallTrackerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
