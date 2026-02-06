#!/usr/bin/env python3
"""
KRSBI-B Soccer Robot - Color Calibrator Node

Interactive HSV color calibration tool for ball, field, and line detection.

Usage:
    ros2 run krsbi_vision color_calibrator
    
Keyboard controls:
    q - Quit
    s - Save current thresholds
    r - Reset to defaults
    1,2,3 - Switch preset (ball, field, line)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String

import cv2
import numpy as np
from cv_bridge import CvBridge
import threading
import json
import os
from typing import Optional, Dict


class ColorCalibratorNode(Node):
    """
    Interactive HSV color calibration node.
    """
    
    def __init__(self):
        super().__init__('color_calibrator')
        
        # =================================================================
        # Parameters
        # =================================================================
        self.declare_parameter('config_path', '')
        self.declare_parameter('camera_topic', '/krsbi/camera/front/image_raw')
        
        self.config_path = self.get_parameter('config_path').value
        camera_topic = self.get_parameter('camera_topic').value
        
        # =================================================================
        # State
        # =================================================================
        self.bridge = CvBridge()
        self.current_frame: Optional[np.ndarray] = None
        self.lock = threading.Lock()
        
        # HSV thresholds
        self.presets = {
            'ball': {
                'h_low': 5, 'h_high': 25,
                's_low': 100, 's_high': 255,
                'v_low': 100, 'v_high': 255,
            },
            'field': {
                'h_low': 35, 'h_high': 85,
                's_low': 50, 's_high': 255,
                'v_low': 50, 'v_high': 255,
            },
            'line': {
                'h_low': 0, 'h_high': 180,
                's_low': 0, 's_high': 50,
                'v_low': 200, 'v_high': 255,
            },
        }
        
        self.current_preset = 'ball'
        self.thresholds = self.presets['ball'].copy()
        
        # Window name
        self.window_name = 'Color Calibrator'
        self.window_created = False
        
        # =================================================================
        # QoS
        # =================================================================
        image_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # =================================================================
        # Subscribers
        # =================================================================
        self.image_sub = self.create_subscription(
            Image, camera_topic,
            self.image_callback, image_qos)
        
        # =================================================================
        # Publishers
        # =================================================================
        self.config_pub = self.create_publisher(
            String, '/krsbi/vision/color_config', 10)
        
        # =================================================================
        # Timer
        # =================================================================
        self.timer = self.create_timer(0.033, self.update)  # 30 fps
        
        self.get_logger().info('Color Calibrator started')
        self.get_logger().info('Press 1=ball, 2=field, 3=line | s=save, r=reset, q=quit')
    
    def image_callback(self, msg: Image):
        """Handle incoming image."""
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            with self.lock:
                self.current_frame = frame
        except Exception as e:
            self.get_logger().error(f'Image error: {e}')
    
    def create_window(self):
        """Create OpenCV window with trackbars."""
        cv2.namedWindow(self.window_name, cv2.WINDOW_AUTOSIZE)
        
        # Create trackbars
        cv2.createTrackbar('H Low', self.window_name, self.thresholds['h_low'], 180, self.on_trackbar)
        cv2.createTrackbar('H High', self.window_name, self.thresholds['h_high'], 180, self.on_trackbar)
        cv2.createTrackbar('S Low', self.window_name, self.thresholds['s_low'], 255, self.on_trackbar)
        cv2.createTrackbar('S High', self.window_name, self.thresholds['s_high'], 255, self.on_trackbar)
        cv2.createTrackbar('V Low', self.window_name, self.thresholds['v_low'], 255, self.on_trackbar)
        cv2.createTrackbar('V High', self.window_name, self.thresholds['v_high'], 255, self.on_trackbar)
        
        self.window_created = True
    
    def on_trackbar(self, val):
        """Trackbar callback."""
        self.thresholds['h_low'] = cv2.getTrackbarPos('H Low', self.window_name)
        self.thresholds['h_high'] = cv2.getTrackbarPos('H High', self.window_name)
        self.thresholds['s_low'] = cv2.getTrackbarPos('S Low', self.window_name)
        self.thresholds['s_high'] = cv2.getTrackbarPos('S High', self.window_name)
        self.thresholds['v_low'] = cv2.getTrackbarPos('V Low', self.window_name)
        self.thresholds['v_high'] = cv2.getTrackbarPos('V High', self.window_name)
    
    def update_trackbars(self):
        """Update trackbar positions from thresholds."""
        if not self.window_created:
            return
        
        cv2.setTrackbarPos('H Low', self.window_name, self.thresholds['h_low'])
        cv2.setTrackbarPos('H High', self.window_name, self.thresholds['h_high'])
        cv2.setTrackbarPos('S Low', self.window_name, self.thresholds['s_low'])
        cv2.setTrackbarPos('S High', self.window_name, self.thresholds['s_high'])
        cv2.setTrackbarPos('V Low', self.window_name, self.thresholds['v_low'])
        cv2.setTrackbarPos('V High', self.window_name, self.thresholds['v_high'])
    
    def update(self):
        """Main update loop."""
        with self.lock:
            frame = self.current_frame.copy() if self.current_frame is not None else None
        
        if frame is None:
            return
        
        # Create window on first frame
        if not self.window_created:
            self.create_window()
        
        # Apply threshold
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        lower = np.array([
            self.thresholds['h_low'],
            self.thresholds['s_low'],
            self.thresholds['v_low'],
        ])
        upper = np.array([
            self.thresholds['h_high'],
            self.thresholds['s_high'],
            self.thresholds['v_high'],
        ])
        
        mask = cv2.inRange(hsv, lower, upper)
        
        # Apply morphology
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        
        # Create visualization
        result = cv2.bitwise_and(frame, frame, mask=mask)
        
        # Compose display
        mask_3ch = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        top_row = np.hstack([frame, hsv])
        bottom_row = np.hstack([mask_3ch, result])
        display = np.vstack([top_row, bottom_row])
        
        # Add text overlay
        info_text = f'Preset: {self.current_preset.upper()}'
        cv2.putText(display, info_text, (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        
        threshold_text = (
            f'H:[{self.thresholds["h_low"]}-{self.thresholds["h_high"]}] '
            f'S:[{self.thresholds["s_low"]}-{self.thresholds["s_high"]}] '
            f'V:[{self.thresholds["v_low"]}-{self.thresholds["v_high"]}]'
        )
        cv2.putText(display, threshold_text, (10, 60),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        # Show
        cv2.imshow(self.window_name, display)
        
        # Handle keyboard
        key = cv2.waitKey(1) & 0xFF
        self.handle_key(key)
    
    def handle_key(self, key: int):
        """Handle keyboard input."""
        if key == ord('q'):
            self.get_logger().info('Quitting...')
            rclpy.shutdown()
        
        elif key == ord('s'):
            self.save_config()
        
        elif key == ord('r'):
            self.reset_preset()
        
        elif key == ord('1'):
            self.switch_preset('ball')
        
        elif key == ord('2'):
            self.switch_preset('field')
        
        elif key == ord('3'):
            self.switch_preset('line')
    
    def switch_preset(self, preset: str):
        """Switch to different preset."""
        if preset not in self.presets:
            return
        
        # Save current to preset
        self.presets[self.current_preset] = self.thresholds.copy()
        
        # Load new preset
        self.current_preset = preset
        self.thresholds = self.presets[preset].copy()
        self.update_trackbars()
        
        self.get_logger().info(f'Switched to preset: {preset}')
    
    def reset_preset(self):
        """Reset current preset to defaults."""
        defaults = {
            'ball': {'h_low': 5, 'h_high': 25, 's_low': 100, 's_high': 255, 'v_low': 100, 'v_high': 255},
            'field': {'h_low': 35, 'h_high': 85, 's_low': 50, 's_high': 255, 'v_low': 50, 'v_high': 255},
            'line': {'h_low': 0, 'h_high': 180, 's_low': 0, 's_high': 50, 'v_low': 200, 'v_high': 255},
        }
        
        self.thresholds = defaults[self.current_preset].copy()
        self.presets[self.current_preset] = self.thresholds.copy()
        self.update_trackbars()
        
        self.get_logger().info(f'Reset preset: {self.current_preset}')
    
    def save_config(self):
        """Save all presets to file."""
        # Update current preset
        self.presets[self.current_preset] = self.thresholds.copy()
        
        config = {
            'presets': self.presets,
            'timestamp': str(self.get_clock().now().to_msg()),
        }
        
        # Save to file
        if self.config_path:
            path = self.config_path
        else:
            path = '/tmp/color_config.json'
        
        try:
            with open(path, 'w') as f:
                json.dump(config, f, indent=2)
            self.get_logger().info(f'Saved config to: {path}')
        except Exception as e:
            self.get_logger().error(f'Save error: {e}')
        
        # Publish config
        msg = String()
        msg.data = json.dumps(config)
        self.config_pub.publish(msg)
        
        # Log values
        self.get_logger().info('Current configuration:')
        for preset, values in self.presets.items():
            self.get_logger().info(f'  {preset}: {values}')
    
    def destroy_node(self):
        """Clean up."""
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    node = ColorCalibratorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
