#!/usr/bin/env python3
"""
KRSBI-B Soccer Robot - Path Planner Node

Simple path planning with obstacle avoidance.
Subscribes to:
    - /move_base_simple/goal (geometry_msgs/PoseStamped)
    - /odom (nav_msgs/Odometry)
    - /krsbi/distance_sensors (krsbi_msgs/DistanceSensors) [Optional]

Publishes:
    - /cmd_vel (geometry_msgs/Twist)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist, Point
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool


import numpy as np
import threading
import math

# Import krsbi_msgs for sensors
try:
    from krsbi_msgs.msg import DistanceSensors
    MSGS_AVAILABLE = True
except ImportError:
    MSGS_AVAILABLE = False

class PathPlanner(Node):
    """
    Simple point-to-point planner with local avoidance.
    """
    
    def __init__(self):
        super().__init__('path_planner')
        
        # =================================================================
        # Parameters
        # =================================================================
        self.declare_parameter('max_vel_x', 1.0)
        self.declare_parameter('max_vel_theta', 2.0)
        self.declare_parameter('goal_tolerance', 0.1)
        self.declare_parameter('angle_tolerance', 0.1)
        
        self.max_v = self.get_parameter('max_vel_x').value
        self.max_w = self.get_parameter('max_vel_theta').value
        self.dist_tol = self.get_parameter('goal_tolerance').value
        self.ang_tol = self.get_parameter('angle_tolerance').value
        
        # =================================================================
        # State
        # =================================================================
        self.goal_x = None
        self.goal_y = None
        self.goal_theta = None
        
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0
        
        self.has_goal = False
        self.obstacle_detected = False
        
        self.lock = threading.Lock()
        
        # =================================================================
        # Publishers & Subscribers
        # =================================================================
        self.cmd_pub = self.create_publisher(
            Twist, '/cmd_vel', 10)
            
        self.status_pub = self.create_publisher(
            Bool, '/path_planner/goal_reached', 10)
            
        self.goal_sub = self.create_subscription(
            PoseStamped, '/move_base_simple/goal',
            self.goal_callback, 10)
            
        self.odom_sub = self.create_subscription(
            Odometry, '/odom',
            self.odom_callback, 10)
            
        if MSGS_AVAILABLE:
            self.sensor_sub = self.create_subscription(
                DistanceSensors, '/krsbi/distance_sensors',
                self.sensor_callback, 10)
        
        # =================================================================
        # Timer
        # =================================================================
        self.timer = self.create_timer(0.05, self.control_loop) # 20Hz
        
        self.get_logger().info('Path Planner started')

    def goal_callback(self, msg: PoseStamped):
        """Handle new goal."""
        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y
        
        # Extract orientation if needed (ignoring for point-to-point, just align at end?)
        # For now, just go to position.
        self.has_goal = True
        self.status_pub.publish(Bool(data=False))
        self.get_logger().info(f'New Goal: ({self.goal_x:.2f}, {self.goal_y:.2f})')

    def odom_callback(self, msg: Odometry):
        """Update robot pose."""
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        # Extract yaw from quaternion
        q = msg.pose.pose.orientation
        # Manual quaternion to yaw
        # yaw = atan2(2(w*z + x*y), 1 - 2(y^2 + z^2))
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_theta = math.atan2(siny_cosp, cosy_cosp)

    def sensor_callback(self, msg):
        """Handle obstacle sensors."""
        # Simple check: stop if very close
        if msg.obstacle_detected and msg.min_distance < 0.2:
            self.obstacle_detected = True
        else:
            self.obstacle_detected = False

    def control_loop(self):
        """Compute velocity command."""
        if not self.has_goal:
            return
            
        dx = self.goal_x - self.current_x
        dy = self.goal_y - self.current_y
        dist = math.sqrt(dx*dx + dy*dy)
        
        # Check if reached
        if dist < self.dist_tol:
            self.has_goal = False
            self.cmd_pub.publish(Twist()) # Stop
            self.status_pub.publish(Bool(data=True))
            self.get_logger().info('Goal Reached')
            return
            
        if self.obstacle_detected:
            # Stop or avoid
            self.cmd_pub.publish(Twist())
            self.get_logger().warn('Obstacle Detected! Stopping.')
            return
            
        # Compute desired velocity (Proportional Control)
        # Global frame velocity
        v_desired = min(dist, self.max_v)
        angle_to_goal = math.atan2(dy, dx)
        
        # Robot frame velocity
        # Vx = V * cos(angle_error)
        # Vy = V * sin(angle_error)
        angle_error = angle_to_goal - self.current_theta
        
        # Normalize angle error
        while angle_error > math.pi: angle_error -= 2*math.pi
        while angle_error < -math.pi: angle_error += 2*math.pi
        
        vx = v_desired * math.cos(angle_error)
        vy = v_desired * math.sin(angle_error)
        
        # Angular control (Face goal)
        w = angle_error * 2.0 # Kp = 2.0
        w = max(min(w, self.max_w), -self.max_w)
        
        # Publish
        cmd = Twist()
        cmd.linear.x = float(vx)
        cmd.linear.y = float(vy)
        cmd.angular.z = float(w)
        
        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = PathPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
