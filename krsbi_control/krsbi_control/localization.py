#!/usr/bin/env python3
"""
KRSBI-B Soccer Robot - Localization Node

Computes odometry from wheel feedback and IMU data.
Publishes /odom and TF odom->base_link.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped, Quaternion, Twist, Pose, Point
from tf2_ros import TransformBroadcaster


import numpy as np
import threading
import time
from typing import Optional

# Import krsbi_msgs
try:
    from krsbi_msgs.msg import MotorFeedback, ImuData
    MSGS_AVAILABLE = True
except ImportError:
    MSGS_AVAILABLE = False

from krsbi_control.omni_kinematics import OmniKinematics3


class LocalizationNode(Node):
    """
    Localization node using odometry and IMU fusion.
    """
    
    def __init__(self):
        super().__init__('localization_node')
        
        # =================================================================
        # Parameters
        # =================================================================
        self.declare_parameter('wheel_radius', 0.05)
        self.declare_parameter('robot_radius', 0.17)
        self.declare_parameter('wheel_angles', [0.0, 120.0, 240.0])
        self.declare_parameter('use_imu', True)
        self.declare_parameter('publish_tf', True)
        
        self.r = self.get_parameter('wheel_radius').value
        self.L = self.get_parameter('robot_radius').value
        self.angles = self.get_parameter('wheel_angles').value
        self.use_imu = self.get_parameter('use_imu').value
        self.publish_tf = self.get_parameter('publish_tf').value
        
        # =================================================================
        # State
        # =================================================================
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        self.vx = 0.0
        self.vy = 0.0
        self.vtheta = 0.0
        
        self.last_time = self.get_clock().now()
        self.lock = threading.Lock()
        
        # Kinematics
        self.kinematics = OmniKinematics3(
            wheel_radius=self.r,
            robot_radius=self.L,
            wheel_angles=self.angles
        )
        
        # TF Broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # =================================================================
        # Publishers
        # =================================================================
        self.odom_pub = self.create_publisher(
            Odometry, '/odom', 10)
        
        # =================================================================
        # Subscribers
        # =================================================================
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        if MSGS_AVAILABLE:
            self.motor_sub = self.create_subscription(
                MotorFeedback, '/krsbi/motor_feedback',
                self.motor_callback, sensor_qos)
            
            self.imu_sub = self.create_subscription(
                ImuData, '/krsbi/imu',
                self.imu_callback, sensor_qos)
        else:
            self.get_logger().error('krsbi_msgs not installed. Localization cannot run.')
        
        self.timer = self.create_timer(0.02, self.update_odometry) # 50Hz
        
        self.get_logger().info('Localization node started')
    
    def motor_callback(self, msg):
        """Handle motor feedback (RPM)."""
        with self.lock:
            # Convert RPM to rad/s
            w1 = self.kinematics.get_wheel_rad_s(msg.motor1_rpm)
            w2 = self.kinematics.get_wheel_rad_s(msg.motor2_rpm)
            w3 = self.kinematics.get_wheel_rad_s(msg.motor3_rpm)
            
            # Forward Kinematics -> Body Velocity
            vx, vy, omega = self.kinematics.forward(w1, w2, w3)
            
            self.vx = vx
            self.vy = vy
            # We use IMU for omega if available, otherwise wheel odometry
            if not self.use_imu:
                self.vtheta = omega
    
    def imu_callback(self, msg):
        """Handle IMU data."""
        if self.use_imu:
            with self.lock:
                # Use Gyro measures directly for angular velocity
                self.vtheta = msg.angular_velocity_z
                # We could also fuse orientation if provided
                # For now, integration of gyro is better for short term, 
                # but absolute orientation from magnetometer is better.
                # Simplified: Just use gyro Z for velocity.
    
    def update_odometry(self):
        """Integrate velocity to position."""
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        
        if dt > 0.1:
            dt = 0.1 # Cap dt if lag occurs
            
        with self.lock:
            # Calculate delta in robot frame
            delta_x = self.vx * dt
            delta_y = self.vy * dt
            delta_theta = self.vtheta * dt
            
            # Transform to world frame
            # Position update: Rotate delta by current theta
            self.x += delta_x * np.cos(self.theta) - delta_y * np.sin(self.theta)
            self.y += delta_x * np.sin(self.theta) + delta_y * np.cos(self.theta)
            self.theta += delta_theta
            
            # Normalize theta
            self.theta = np.arctan2(np.sin(self.theta), np.cos(self.theta))
            
            # Prepare Odometry message
            odom_msg = Odometry()
            odom_msg.header.stamp = current_time.to_msg()
            odom_msg.header.frame_id = 'odom'
            odom_msg.child_frame_id = 'base_footprint'
            
            # Position
            odom_msg.pose.pose.position.x = self.x
            odom_msg.pose.pose.position.y = self.y
            odom_msg.pose.pose.position.z = 0.0
            
            # Orientation
            # q = tf_transformations.quaternion_from_euler(0, 0, self.theta)
            # quaternion for Z-rotation: [0, 0, sin(theta/2), cos(theta/2)]
            qz = np.sin(self.theta / 2.0)
            qw = np.cos(self.theta / 2.0)
            odom_msg.pose.pose.orientation = Quaternion(x=0.0, y=0.0, z= float(qz), w= float(qw))
            
            # Velocity (in child frame / base_link)
            odom_msg.twist.twist.linear.x = self.vx
            odom_msg.twist.twist.linear.y = self.vy
            odom_msg.twist.twist.angular.z = self.vtheta
            
            # Covariance (simplified)
            odom_msg.pose.covariance[0] = 0.01
            odom_msg.pose.covariance[7] = 0.01
            odom_msg.pose.covariance[35] = 0.01
            
            self.odom_pub.publish(odom_msg)
            
            # Publish TF
            if self.publish_tf:
                t = TransformStamped()
                t.header.stamp = current_time.to_msg()
                t.header.frame_id = 'odom'
                t.child_frame_id = 'base_footprint'
                
                t.transform.translation.x = self.x
                t.transform.translation.y = self.y
                t.transform.translation.z = 0.0
                t.transform.rotation = odom_msg.pose.pose.orientation
                
                self.tf_broadcaster.sendTransform(t)
        
        self.last_time = current_time

def main(args=None):
    rclpy.init(args=args)
    
    node = LocalizationNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
