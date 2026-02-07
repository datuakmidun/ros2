import py_trees
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import String
from rclpy.node import Node
import math

class Action(py_trees.behaviour.Behaviour):
    def __init__(self, name="Action", node: Node = None):
        super(Action, self).__init__(name)
        self.node = node
        self.cmd_pub = None
        self.behavior_pub = None
        
    def setup(self, **kwargs):
        if self.node:
            self.cmd_pub = self.node.create_publisher(Twist, '/cmd_vel', 10)
            self.behavior_pub = self.node.create_publisher(String, '/krsbi/behavior/command', 10)
        return True

    def send_cmd(self, vx, vy, w):
        if self.cmd_pub:
            msg = Twist()
            msg.linear.x = float(vx)
            msg.linear.y = float(vy)
            msg.angular.z = float(w)
            self.cmd_pub.publish(msg)

    def send_behavior(self, command):
        if self.behavior_pub:
            msg = String()
            msg.data = command
            self.behavior_pub.publish(msg)

class GoToPosition(Action):
    def __init__(self, name="GoToPosition", node=None, x=0.0, y=0.0):
        super(GoToPosition, self).__init__(name, node)
        self.target_x = x
        self.target_y = y
        self.goal_pub = None
        
    def setup(self, **kwargs):
        super().setup(**kwargs)
        if self.node:
            self.goal_pub = self.node.create_publisher(PoseStamped, '/move_base_simple/goal', 10)
        return True

    def initialise(self):
        # Send goal once
        if self.goal_pub:
            msg = PoseStamped()
            msg.header.frame_id = "odom" # Global frame
            msg.header.stamp = self.node.get_clock().now().to_msg()
            msg.pose.position.x = self.target_x
            msg.pose.position.y = self.target_y
            # Orientation 0 for now
            self.goal_pub.publish(msg)
            
    def update(self):
        # We assume path planner handles movement.
        # Check distance to goal in blackboard
        bb = py_trees.blackboard.Blackboard()
        robot_pose = bb.get("world_state.robot_pose")
        
        if not robot_pose:
            return py_trees.common.Status.RUNNING
            
        dx = self.target_x - robot_pose.x
        dy = self.target_y - robot_pose.y
        dist = math.sqrt(dx*dx + dy*dy)
        if dist < 0.1:
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING

class FollowBall(Action):
    def __init__(self, name="FollowBall", node=None):
        super(FollowBall, self).__init__(name, node)
        
    def  initialise(self):
        self.send_behavior("FOLLOW_BALL")
        
    def update(self):
        # Check if ball visible
        bb = py_trees.blackboard.Blackboard()
        ball = bb.get("world_state.ball")
        if ball and ball.is_visible:
            return py_trees.common.Status.RUNNING
        else:
            return py_trees.common.Status.FAILURE
            
    def terminate(self, new_status):
        self.send_behavior("IDLE")

class KickBall(Action):
    def __init__(self, name="KickBall", node=None):
        super(KickBall, self).__init__(name, node)
        self.kick_pub = None
        
    def setup(self, **kwargs):
        super().setup(**kwargs)
        # Assuming krsbi_comm service or topic
        # Using topic for simplicity here
        # Actually krsbi_comm has /krsbi/kick service
        return True

    def update(self):
        # Send kick command once
        self.node.get_logger().info("KICK!")
        # Implement service call via node client if needed
        # For prototype, assume separate Kick node handles /krsbi/kick
        # Or publish to behavior
        self.send_behavior("KICK")
        return py_trees.common.Status.SUCCESS

class RotateToGoal(Action):
    def __init__(self, name="RotateToGoal", node=None):
        super(RotateToGoal, self).__init__(name, node)
        
    def update(self):
        # Simple P-control to align with opponent goal (6, 0)
        bb = py_trees.blackboard.Blackboard()
        pose = bb.get("world_state.robot_pose")
        
        if not pose:
            return py_trees.common.Status.FAILURE
            
        goal_x, goal_y = (6.0, 0.0) # Opponent goal
        dx = goal_x - pose.x
        dy = goal_y - pose.y
        target_angle = math.atan2(dy, dx)
        
        diff = target_angle - pose.theta
        while diff > math.pi: diff -= 2*math.pi
        while diff < -math.pi: diff += 2*math.pi
        
        if abs(diff) < 0.1:
            self.send_cmd(0, 0, 0) # Stop
            return py_trees.common.Status.SUCCESS
        
        # Turn
        w = 2.0 * diff
        w = max(min(w, 2.0), -2.0)
        self.send_cmd(0, 0, w)
        return py_trees.common.Status.RUNNING
