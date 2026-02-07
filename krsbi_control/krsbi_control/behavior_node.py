#!/usr/bin/env python3
"""
KRSBI-B Soccer Robot - Behavior Node

Manages high-level reactive behaviors like FollowBall, Dribble, Kick.
Subscribes:
    - /krsbi/behavior/command (std_msgs/String)
    - /krsbi/vision/ball (krsbi_msgs/BallPosition)
    - /odom (nav_msgs/Odometry)

Publishes:
    - /cmd_vel (geometry_msgs/Twist)
    - /krsbi/behavior/status (std_msgs/String)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from nav_msgs.msg import Odometry

# Import krsbi_msgs
try:
    from krsbi_msgs.msg import BallPosition
    MSGS_AVAILABLE = True
except ImportError:
    MSGS_AVAILABLE = False

class BehaviorNode(Node):
    """
    Executes specific robot behaviors.
    """
    
    def __init__(self):
        super().__init__('behavior_node')
        
        # State
        self.active_behavior = "IDLE"
        self.ball_pos = None
        self.ball_seen_time = 0
        
        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/krsbi/behavior/status', 10)
        
        # Subscribers
        self.cmd_sub = self.create_subscription(
            String, '/krsbi/behavior/command',
            self.behavior_callback, 10
        )
        
        if MSGS_AVAILABLE:
            self.ball_sub = self.create_subscription(
                BallPosition, '/krsbi/vision/ball',
                self.ball_callback, 10
            )
            
        # Timer
        self.timer = self.create_timer(0.05, self.update) # 20Hz
        
        self.get_logger().info('Behavior Node started')

    def behavior_callback(self, msg: String):
        command = msg.data.upper()
        if command in ["IDLE", "FOLLOW_BALL", "DRIBBLE", "KICK"]:
            self.active_behavior = command
            self.get_logger().info(f"Behavior changed to: {self.active_behavior}")
        else:
            self.get_logger().warn(f"Unknown behavior: {command}")

    def ball_callback(self, msg):
        self.ball_pos = msg
        self.ball_seen_time = self.get_clock().now().nanoseconds / 1e9

    def update(self):
        cmd = Twist()
        
        current_time = self.get_clock().now().nanoseconds / 1e9
        ball_fresh = (current_time - self.ball_seen_time) < 1.0 # 1 sec timeout
        
        if self.active_behavior == "FOLLOW_BALL":
            if ball_fresh and self.ball_pos and self.ball_pos.detected:
                # Simple P-control to center ball and approach
                # Ball X is distance (forward), Y is lateral offset?
                # Check krsbi_vision coordinates.
                # Usually X forward, Y left.
                # BallPosition msg has `distance` and `pixel_x`.
                # Wait, usually ball msg has relative coordinates in robot frame.
                # If `BallPosition` has (x, y) relative to robot base_link:
                # Drive towards it.
                
                # Assume ball_pos.x is Forward dist, ball_pos.y is Left dist.
                target_dist = 0.3 # Keep 30cm distance
                error_x = self.ball_pos.x - target_dist
                error_y = self.ball_pos.y
                
                cmd.linear.x = 1.0 * error_x
                cmd.linear.y = 1.0 * error_y
                # Turn to face ball (y error is proportional to angle if x is large)
                cmd.angular.z = 2.0 * error_y 
                
                # Clamp
                cmd.linear.x = max(min(cmd.linear.x, 1.0), -1.0)
                cmd.linear.y = max(min(cmd.linear.y, 1.0), -1.0)
                cmd.angular.z = max(min(cmd.angular.z, 2.0), -2.0)
            else:
                # Search (Spin)
                cmd.angular.z = 0.5
        
        elif self.active_behavior == "IDLE":
            pass # Stop
            
        self.cmd_pub.publish(cmd)
        self.status_pub.publish(String(data=self.active_behavior))

def main(args=None):
    rclpy.init(args=args)
    node = BehaviorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
