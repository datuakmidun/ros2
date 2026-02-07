#!/usr/bin/env python3
"""
KRSBI-B Ball Seeker Node

Implements the main strategy state machine:
1. SEARCHING: Rotate to find ball (using last known position/angle)
   - If lost close to robot -> Check blind spot / back up
   - If lost far away -> Rotate towards last known angle
2. ALIGNING (Omni): Use Omni camera to align ball to 0 degrees (front)
3. APPROACHING (Front): Use Front camera to approach/estimate distance
4. DRIBBLING (Sharp GP): Blind spot handling when ball is very close

Subscribes:
    - /krsbi/vision/ball (BallPosition)
    - /krsbi/sensor/proximity (Bool) - proximity sensor (Sharp GP)
    
Publishes:
    - /krsbi/cmd_vel (Twist) - robot control
    - /krsbi/strategy/state (String) - current state debug
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool
from krsbi_msgs.msg import BallPosition
import math
import time

# States
STATE_SEARCHING = "SEARCHING"
STATE_ALIGNING = "ALIGNING (OMNI)"
STATE_APPROACHING = "APPROACHING (FRONT)"
STATE_DRIBBLING = "DRIBBLING (SENSOR)"

class BallSeekerNode(Node):
    def __init__(self):
        super().__init__('ball_seeker_node')
        
        # Parameters
        self.declare_parameter('align_tolerance', 0.1)     # radians (~5.7 degrees)
        self.declare_parameter('target_distance', 0.2)     # meters (stop distance)
        
        # Control Gains
        self.kp_align = 1.5   # Omni rotation gain
        self.kp_center = 1.0  # Front camera centering gain
        self.kp_dist = 0.8    # Distance approach gain
        self.max_rot = 2.0
        self.max_lin = 0.5
        
        # State Variables
        self.state = STATE_SEARCHING
        self.last_ball_msg = None
        self.last_seen_time = 0
        self.ball_visible = False
        self.proximity_active = False # Sharp GP
        
        # Memory
        self.last_known_angle = 0.0
        self.last_known_dist = 10.0
        
        # Subscribers
        self.ball_sub = self.create_subscription(
            BallPosition, '/krsbi/vision/ball', self.ball_callback, 10
        )
        self.prox_sub = self.create_subscription(
            Bool, '/krsbi/sensor/proximity', self.prox_callback, 10
        )
            
        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/krsbi/cmd_vel', 10)
        self.state_pub = self.create_publisher(String, '/krsbi/strategy/state', 10)
        
        # Timer (Control Loop 20Hz)
        self.timer = self.create_timer(0.05, self.control_loop)
        
        self.get_logger().info('Ball Seeker Strategy Initiated')

    def ball_callback(self, msg):
        # Only process if visible
        if msg.is_visible:
            self.ball_visible = True
            self.last_ball_msg = msg
            self.last_seen_time = time.time()
            
            # Update memory
            self.last_known_angle = msg.angle
            self.last_known_dist = msg.distance

    def prox_callback(self, msg):
        self.proximity_active = msg.data

    def control_loop(self):
        # 1. Update Visibility Status (Timeout 0.5s)
        if time.time() - self.last_seen_time > 0.5:
            self.ball_visible = False
            
        # 2. Determine State
        self.determine_state()
        
        # 3. Calculate & Publish Commands
        cmd = self.compute_command()
        self.cmd_pub.publish(cmd)
        
        # 4. Debug State
        self.state_pub.publish(String(data=self.state))

    def determine_state(self):
        # Priority 1: Physical Sensor (Blind Spot)
        if self.proximity_active:
            self.state = STATE_DRIBBLING
            return

        # Priority 2: Visual Detection
        if self.ball_visible:
            if self.last_ball_msg.camera_source == BallPosition.CAMERA_FRONT:
                self.state = STATE_APPROACHING
            elif self.last_ball_msg.camera_source == BallPosition.CAMERA_OMNI:
                self.state = STATE_ALIGNING
            else:
                self.state = STATE_ALIGNING # Fallback
            return

        # Priority 3: Lost -> Search
        self.state = STATE_SEARCHING

    def compute_command(self):
        cmd = Twist()
        
        if self.state == STATE_DRIBBLING:
            # Ball is detained by dribbler/sensor
            # Action: Move forward carefully or execute kick
            cmd.linear.x = 0.3
            self.get_logger().info("DRIBBLING ACTIVE")
            
        elif self.state == STATE_APPROACHING:
            # Source: Front Camera
            # Goal: Maintain distance & center
            dist_err = self.last_ball_msg.distance - self.get_parameter('target_distance').value
            angle_err = self.last_ball_msg.angle
            
            cmd.linear.x = min(max(dist_err * self.kp_dist, -self.max_lin), self.max_lin)
            cmd.angular.z = min(max(angle_err * self.kp_center, -self.max_rot), self.max_rot)
            
        elif self.state == STATE_ALIGNING:
            # Source: Omni Camera
            # Goal: Rotate to bring ball to front (0 rad).
            angle_err = self.last_ball_msg.angle
            
            # PID Rotation
            cmd.angular.z = min(max(angle_err * self.kp_align, -self.max_rot), self.max_rot)
            
            # If reasonably aligned (< 20 deg), start approaching slowly
            if abs(angle_err) < 0.35:
                cmd.linear.x = 0.2
            
        elif self.state == STATE_SEARCHING:
            # Logic when ball is lost
            # Case A: Lost very close (likely under robot or just barely out of view)
            if self.last_known_dist < 0.4:
                # Back up slightly and scan
                cmd.linear.x = -0.1
                # Spin slowly
                cmd.angular.z = 0.5 if self.last_known_angle > 0 else -0.5
                
            # Case B: Lost far away (out of frame)
            else:
                # Rotate towards last known direction
                if self.last_known_angle > 0:
                    cmd.angular.z = 1.0  # Turn Left
                else:
                    cmd.angular.z = -1.0 # Turn Right
                    
        return cmd

def main(args=None):
    rclpy.init(args=args)
    node = BallSeekerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
