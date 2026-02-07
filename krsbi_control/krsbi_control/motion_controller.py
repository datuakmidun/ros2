#!/usr/bin/env python3
"""
KRSBI-B Soccer Robot - Motion Controller Node

Handles velocity commands, applies limits and ramping, and
publishes to low-level communication.

Subscribes:
    - /cmd_vel (geometry_msgs/Twist)
    
Publishes:
    - /krsbi/cmd_vel (geometry_msgs/Twist) - Safe/Ramped velocity
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
import numpy as np

class MotionController(Node):
    """
    Motion controller with velocity ramping and safety limits.
    """
    
    def __init__(self):
        super().__init__('motion_controller')
        
        # =================================================================
        # Parameters
        # =================================================================
        self.declare_parameter('max_vel_x', 2.0)
        self.declare_parameter('max_vel_y', 2.0)
        self.declare_parameter('max_vel_theta', 4.0)
        self.declare_parameter('acc_limit_linear', 2.0)
        self.declare_parameter('acc_limit_angular', 6.0)
        self.declare_parameter('frequency', 50.0)
        self.declare_parameter('timeout', 0.5)
        
        self.max_vel_x = self.get_parameter('max_vel_x').value
        self.max_vel_y = self.get_parameter('max_vel_y').value
        self.max_vel_theta = self.get_parameter('max_vel_theta').value
        self.acc_lin = self.get_parameter('acc_limit_linear').value
        self.acc_ang = self.get_parameter('acc_limit_angular').value
        self.freq = self.get_parameter('frequency').value
        self.timeout = self.get_parameter('timeout').value
        
        # =================================================================
        # State
        # =================================================================
        self.target_vx = 0.0
        self.target_vy = 0.0
        self.target_w = 0.0
        
        self.current_vx = 0.0
        self.current_vy = 0.0
        self.current_w = 0.0
        
        self.last_cmd_time = self.get_clock().now()
        self.last_update_time = self.get_clock().now()
        
        # =================================================================
        # Publishers & Subscribers
        # =================================================================
        self.cmd_pub = self.create_publisher(
            Twist, '/krsbi/cmd_vel', 10)
            
        self.cmd_sub = self.create_subscription(
            Twist, '/cmd_vel',
            self.cmd_callback, 10)
            
        # =================================================================
        # Timer
        # =================================================================
        self.timer = self.create_timer(1.0/self.freq, self.update)
        
        self.get_logger().info('Motion Controller started')

    def cmd_callback(self, msg: Twist):
        """Handle incoming velocity command."""
        self.target_vx = np.clip(msg.linear.x, -self.max_vel_x, self.max_vel_x)
        self.target_vy = np.clip(msg.linear.y, -self.max_vel_y, self.max_vel_y)
        self.target_w = np.clip(msg.angular.z, -self.max_vel_theta, self.max_vel_theta)
        
        self.last_cmd_time = self.get_clock().now()

    def update(self):
        """Update loop for ramping and publishing."""
        current_time = self.get_clock().now()
        dt = (current_time - self.last_update_time).nanoseconds / 1e9
        
        if dt <= 0:
            return
            
        # Check timeout
        time_since_cmd = (current_time - self.last_cmd_time).nanoseconds / 1e9
        if time_since_cmd > self.timeout:
            self.target_vx = 0.0
            self.target_vy = 0.0
            self.target_w = 0.0
            
        # Ramping
        max_delta_v = self.acc_lin * dt
        max_delta_w = self.acc_ang * dt
        
        self.current_vx = self.ramp(self.current_vx, self.target_vx, max_delta_v)
        self.current_vy = self.ramp(self.current_vy, self.target_vy, max_delta_v)
        self.current_w = self.ramp(self.current_w, self.target_w, max_delta_w)
        
        # Publish
        msg = Twist()
        msg.linear.x = float(self.current_vx)
        msg.linear.y = float(self.current_vy)
        msg.angular.z = float(self.current_w)
        
        self.cmd_pub.publish(msg)
        self.last_update_time = current_time

    def ramp(self, current: float, target: float, max_delta: float) -> float:
        """Ramp value towards target."""
        delta = target - current
        if abs(delta) <= max_delta:
            return target
        else:
            return current + np.sign(delta) * max_delta

def main(args=None):
    rclpy.init(args=args)
    node = MotionController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
