#!/usr/bin/env python3
"""
KRSBI-B Vision - Dual Camera Ball Detection Launch

Runs both front and omni cameras with ball detection.

Usage:
    ros2 launch krsbi_vision dual_camera.launch.py
    ros2 launch krsbi_vision dual_camera.launch.py front_device:=1 omni_device:=0
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package share directory
    pkg_share = get_package_share_directory('krsbi_vision')
    
    # Default model path
    default_model = os.path.join(pkg_share, 'models', 'yolov8s.pt')
    
    # =========================================================================
    # Launch Arguments
    # =========================================================================
    front_device_arg = DeclareLaunchArgument(
        'front_device', default_value='1',
        description='Front camera device ID (/dev/videoX)')
    
    omni_device_arg = DeclareLaunchArgument(
        'omni_device', default_value='0',
        description='Omni camera device ID (/dev/videoX)')
    
    model_arg = DeclareLaunchArgument(
        'model', default_value=default_model,
        description='YOLO model path')
    
    ball_class_arg = DeclareLaunchArgument(
        'ball_class', default_value='0',
        description='Ball class ID in YOLO model')
    
    # =========================================================================
    # Front Camera Node
    # =========================================================================
    front_camera = Node(
        package='krsbi_vision',
        executable='camera_node',
        name='front_camera',
        namespace='krsbi',
        output='screen',
        parameters=[{
            'device_id': LaunchConfiguration('front_device'),
            'width': 640,
            'height': 480,
            'fps': 30,
            'frame_id': 'front_camera_link',
        }],
        remappings=[
            ('image_raw', 'camera/front/image_raw'),
            ('camera_info', 'camera/front/camera_info'),
        ]
    )
    
    # =========================================================================
    # Omni Camera Node (360° fisheye)
    # =========================================================================
    omni_camera = Node(
        package='krsbi_vision',
        executable='omni_camera_node',
        name='omni_camera',
        namespace='krsbi',
        output='screen',
        parameters=[{
            'device_id': LaunchConfiguration('omni_device'),
            'width': 640,
            'height': 480,
            'fps': 30,
            'frame_id': 'omni_camera_link',
            # Fisheye parameters
            'mirror_center_x': 320,
            'mirror_center_y': 240,
            'inner_radius': 60,
            'outer_radius': 230,
            'unwrap_width': 640,
            'unwrap_height': 120,
        }],
    )
    
    # =========================================================================
    # Ball Detector (subscribes to both cameras)
    # =========================================================================
    ball_detector = Node(
        package='krsbi_vision',
        executable='ball_detector',
        name='ball_detector',
        namespace='krsbi',
        output='screen',
        parameters=[{
            'use_yolo': True,
            'use_color': True,
            'yolo_model': LaunchConfiguration('model'),
            'ball_class_id': LaunchConfiguration('ball_class'),
            'yolo_confidence': 0.5,
            'publish_debug': True,
            'detection_rate': 20.0,
        }]
    )
    
    # =========================================================================
    # Log Info
    # =========================================================================
    log_info = LogInfo(msg='Starting Dual Camera Vision System...')
    
    return LaunchDescription([
        # Arguments
        front_device_arg,
        omni_device_arg,
        model_arg,
        ball_class_arg,
        
        # Info
        log_info,
        
        # Nodes
        front_camera,
        omni_camera,
        ball_detector,
    ])
