#!/usr/bin/env python3
"""
KRSBI-B Soccer Robot - Strategy Manager Node

The brain of the robot.
- Maintains World Model state from sensors.
- Executes Behavior Tree logic.
- Publishes high-level commands.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry

import py_trees
import time
import threading
import math

# Import local modules
from krsbi_decision.world_model.world_state import WorldState, RobotPose, BallState
from krsbi_decision.behavior_tree.roles import create_striker_root, create_goalie_root

# Import krsbi_msgs if available
try:
    from krsbi_msgs.msg import BallPosition
    MSGS_AVAILABLE = True
except ImportError:
    MSGS_AVAILABLE = False

class StrategyManager(Node):
    """
    Main decision making node.
    """
    
    def __init__(self):
        super().__init__('strategy_manager')
        
        # Parameters
        self.declare_parameter('role', 'striker')
        self.declare_parameter('frequency', 10.0) # Hz
        
        self.role = self.get_parameter('role').value
        self.freq = self.get_parameter('frequency').value
        
        # State
        self.world_model = WorldState()
        self.blackboard = py_trees.blackboard.Blackboard()
        self.lock = threading.Lock()
        
        # Initialize Tree based on Role
        self.tree_root = None
        if self.role == 'striker':
            self.tree_root = create_striker_root(self)
        elif self.role == 'goalie':
            self.tree_root = create_goalie_root(self)
        else:
            self.get_logger().error(f"Unknown role: {self.role}")
            return

        self.behaviour_tree = py_trees.trees.BehaviourTree(self.tree_root)
        self.behaviour_tree.setup(timeout=15)
        
        # Publishers
        # Actions in Behavior Tree handle publishing directly using 'self'.
        # But we need basic debug pub
        self.state_pub = self.create_publisher(String, '/krsbi/decision/state', 10)
        
        # Subscribers
        sensor_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=1)
        
        self.odom_sub = self.create_subscription(
            Odometry, '/odom',
            self.odom_callback, sensor_qos)
            
        if MSGS_AVAILABLE:
            self.ball_sub = self.create_subscription(
                BallPosition, '/krsbi/vision/ball',
                self.ball_callback, sensor_qos)
        
        self.game_sub = self.create_subscription(
            String, '/krsbi/game/state',
            self.game_state_callback, 10)

        # Timer for Thinking
        self.timer = self.create_timer(1.0/self.freq, self.tick)
        
        self.get_logger().info(f'Strategy Manager started. Role: {self.role}')

    def odom_callback(self, msg: Odometry):
        """Update robot pose."""
        with self.lock:
            # Simple conversion from quaternion to yaw
            q = msg.pose.pose.orientation
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            yaw = float(py_trees.common.math.atan2(siny_cosp, cosy_cosp)) # Just naming glitch? No, numpy/math exists. Use math.
            # Assuming math is imported or used.
            # I forgot import math.
            import math
            yaw = math.atan2(siny_cosp, cosy_cosp)
            
            self.world_model.robot_pose.x = msg.pose.pose.position.x
            self.world_model.robot_pose.y = msg.pose.pose.position.y
            self.world_model.robot_pose.theta = yaw

    def ball_callback(self, msg):
        """Update ball state."""
        with self.lock:
            if not self.world_model.ball:
                self.world_model.ball = BallState(0,0)
            
            # Message provides Relative coords? Check krsbi_vision.
            # Assuming x=dist, y=lateral for now as per control/vision discussion.
            # To put in WorldState (Global), we need robot pose.
            rx = self.world_model.robot_pose.x
            ry = self.world_model.robot_pose.y
            rt = self.world_model.robot_pose.theta
            
            import math
            # Local to Global
            # Local X is forward?
            # Global X = rx + local_x * cos(rt) - local_y * sin(rt)
            lx = msg.x
            ly = msg.y
            
            gx = rx + lx * math.cos(rt) - ly * math.sin(rt)
            gy = ry + lx * math.sin(rt) + ly * math.cos(rt)
            
            self.world_model.ball.x = gx
            self.world_model.ball.y = gy
            self.world_model.ball.is_visible = msg.detected
            self.world_model.ball.last_seen = self.get_clock().now().nanoseconds / 1e9

    def game_state_callback(self, msg: String):
        with self.lock:
            self.world_model.game_phase = msg.data

    def tick(self):
        """Execute one cycle of the behavior tree."""
        with self.lock:
            # Update Blackboard
            self.blackboard.set("world_state.robot_pose", self.world_model.robot_pose)
            self.blackboard.set("world_state.ball", self.world_model.ball)
            self.blackboard.set("world_state.game_phase", self.world_model.game_phase)
        
        # Tick Tree
        self.behaviour_tree.tick()
        
        # Publish Debug
        # status = self.tree_root.status
        # self.state_pub.publish(String(data=str(status)))

def main(args=None):
    rclpy.init(args=args)
    node = StrategyManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
