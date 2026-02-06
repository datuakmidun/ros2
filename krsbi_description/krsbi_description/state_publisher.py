#!/usr/bin/env python3
"""
KRSBI-B Robot - Custom State Publisher

This node publishes dynamic joint states based on sensor feedback from krsbi_comm.
It bridges the gap between Arduino sensor data and ROS 2 joint states.

Subscribes to:
    - /krsbi/motor_feedback (krsbi_msgs/MotorFeedback)
    - /krsbi/gripper_state (krsbi_msgs/GripperState)
    - /krsbi/kicker_state (krsbi_msgs/KickerState)

Publishes to:
    - /joint_states (sensor_msgs/JointState)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math

# Try to import krsbi_msgs, fall back to simulation mode if not available
try:
    from krsbi_msgs.msg import MotorFeedback, GripperState, KickerState
    MSGS_AVAILABLE = True
except ImportError:
    MSGS_AVAILABLE = False
    print("Warning: krsbi_msgs not found. Running in simulation mode.")


class KrsbiStatePublisher(Node):
    """
    Custom joint state publisher for KRSBI-B robot.
    
    Converts sensor feedback to joint states for robot_state_publisher.
    """
    
    def __init__(self):
        super().__init__('krsbi_state_publisher')
        
        # Parameters
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('simulation_mode', not MSGS_AVAILABLE)
        self.declare_parameter('wheel_radius', 0.05)
        
        self.publish_rate = self.get_parameter('publish_rate').value
        self.simulation_mode = self.get_parameter('simulation_mode').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        
        # Joint names
        self.wheel_joints = [
            'front_wheel_joint',
            'rear_left_wheel_joint',
            'rear_right_wheel_joint',
        ]
        self.gripper_joints = [
            'gripper_left_joint',
            'gripper_right_joint',
        ]
        self.kicker_joints = [
            'kicker_joint',
        ]
        self.all_joints = self.wheel_joints + self.gripper_joints + self.kicker_joints
        
        # Joint states
        self.joint_positions = {name: 0.0 for name in self.all_joints}
        self.joint_velocities = {name: 0.0 for name in self.all_joints}
        self.joint_efforts = {name: 0.0 for name in self.all_joints}
        
        # Wheel position tracking (cumulative)
        self.wheel_positions = [0.0, 0.0, 0.0]
        self.last_time = self.get_clock().now()
        
        # Publisher
        self.joint_state_pub = self.create_publisher(
            JointState,
            'joint_states',
            10
        )
        
        # Subscribers (if messages available)
        if MSGS_AVAILABLE and not self.simulation_mode:
            self.motor_sub = self.create_subscription(
                MotorFeedback,
                '/krsbi/motor_feedback',
                self.motor_feedback_callback,
                10
            )
            self.gripper_sub = self.create_subscription(
                GripperState,
                '/krsbi/gripper_state',
                self.gripper_state_callback,
                10
            )
            self.kicker_sub = self.create_subscription(
                KickerState,
                '/krsbi/kicker_state',
                self.kicker_state_callback,
                10
            )
            self.get_logger().info('Subscribed to sensor feedback topics')
        else:
            self.get_logger().info('Running in simulation mode (no sensor feedback)')
        
        # Timer for publishing
        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.publish_joint_states)
        
        self.get_logger().info(f'KRSBI State Publisher started at {self.publish_rate} Hz')
    
    def motor_feedback_callback(self, msg: 'MotorFeedback'):
        """Process motor feedback and update wheel joint states."""
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time
        
        # Convert RPM to rad/s
        rpm_to_rads = 2 * math.pi / 60
        wheel_velocities = [
            msg.motor1_rpm * rpm_to_rads,
            msg.motor2_rpm * rpm_to_rads,
            msg.motor3_rpm * rpm_to_rads,
        ]
        
        # Update positions (integrate velocity)
        for i, joint in enumerate(self.wheel_joints):
            self.wheel_positions[i] += wheel_velocities[i] * dt
            # Normalize to [-pi, pi] for display purposes
            self.joint_positions[joint] = self.wheel_positions[i] % (2 * math.pi)
            self.joint_velocities[joint] = wheel_velocities[i]
    
    def gripper_state_callback(self, msg: 'GripperState'):
        """Process gripper state and update gripper joint positions."""
        # Map gripper position (0-1) to joint limits (0-0.03)
        gripper_position = msg.position * 0.03  # Max 3cm opening
        
        self.joint_positions['gripper_left_joint'] = gripper_position
        self.joint_positions['gripper_right_joint'] = gripper_position
    
    def kicker_state_callback(self, msg: 'KickerState'):
        """Process kicker state for visualization."""
        # Kicker is normally retracted (0), extended when kicking
        if msg.state == 3:  # STATE_KICKING
            # Animate kicker extension
            self.joint_positions['kicker_joint'] = 0.05  # Max extension
        else:
            self.joint_positions['kicker_joint'] = 0.0
    
    def publish_joint_states(self):
        """Publish current joint states."""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = ''
        
        msg.name = self.all_joints
        msg.position = [self.joint_positions[name] for name in self.all_joints]
        msg.velocity = [self.joint_velocities[name] for name in self.all_joints]
        msg.effort = [self.joint_efforts[name] for name in self.all_joints]
        
        self.joint_state_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    
    node = KrsbiStatePublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
