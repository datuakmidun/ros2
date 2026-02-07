#!/usr/bin/env python3
"""
KRSBI-B Soccer Robot - Game Controller

Manages game state transitions and referee commands.
Subscribes:
    - /referee (std_msgs/String) - e.g. "START", "STOP", "GOAL_BLUE"

Publishes:
    - /krsbi/game/state (std_msgs/String) - e.g. "PLAYING", "HALTED"
    - /krsbi/game/score (std_msgs/Int32MultiArray) - [blue, yellow]
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32MultiArray

class GameController(Node):
    def __init__(self):
        super().__init__('game_controller')
        
        self.state = "HALTED"
        self.score_blue = 0
        self.score_yellow = 0
        
        self.state_pub = self.create_publisher(String, '/krsbi/game/state', 10)
        self.score_pub = self.create_publisher(Int32MultiArray, '/krsbi/game/score', 10)
        
        self.ref_sub = self.create_subscription(
            String, '/referee',
            self.ref_callback, 10
        )
        
        self.timer = self.create_timer(1.0, self.publish_state)
        
        self.get_logger().info('Game Controller started')

    def ref_callback(self, msg):
        cmd = msg.data.upper()
        
        if cmd == "START":
            self.state = "PLAYING"
        elif cmd == "STOP":
            self.state = "HALTED"
        elif cmd == "READY":
            self.state = "READY"
        elif cmd == "SET":
            self.state = "SET"
        elif cmd.startswith("GOAL_BLUE"):
            self.score_blue += 1
            self.state = "HALTED" # Usually halts after goal
        elif cmd.startswith("GOAL_YELLOW"):
            self.score_yellow += 1
            self.state = "HALTED"
        else:
            self.get_logger().warn(f"Unknown ref command: {cmd}")
            
        self.get_logger().info(f"Game State: {self.state}")

    def publish_state(self):
        self.state_pub.publish(String(data=self.state))
        
        score_msg = Int32MultiArray()
        score_msg.data = [self.score_blue, self.score_yellow]
        self.score_pub.publish(score_msg)

def main(args=None):
    rclpy.init(args=args)
    node = GameController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
