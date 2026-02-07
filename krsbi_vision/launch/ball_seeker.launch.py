#!/usr/bin/env python3
"""
KRSBI-B Ball Seeker Launch

Launches the complete ball seeking system:
1. Omni Camera (Device 0) - For finding ball direction
2. Front Camera (Device 1) - For precision approach
3. Ball Detector - Detects ball in both cameras
4. Ball Seeker - State machine strategy

Usage:
    ros2 launch krsbi_vision ball_seeker.launch.py
    ros2 launch krsbi_vision ball_seeker.launch.py model:=/path/to/custom.pt
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('krsbi_vision')
    default_model = os.path.join(pkg_share, 'models', 'yolov8s.pt')
    
    # Arguments
    omni_dev_arg = DeclareLaunchArgument('omni_device', default_value='0')
    front_dev_arg = DeclareLaunchArgument('front_device', default_value='1')
    model_arg = DeclareLaunchArgument('model', default_value=default_model)
    class_arg = DeclareLaunchArgument('ball_class', default_value='0')
    
    # 1. Omni Camera
    omni_cam = Node(
        package='krsbi_vision',
        executable='camera_node',
        name='omni_camera',
        namespace='krsbi',
        output='screen',
        parameters=[{
            'device_id': LaunchConfiguration('omni_device'),
            'width': 640, 'height': 480, 'fps': 30
        }],
        remappings=[('image_raw', 'camera/omni/image_raw')]
    )
    
    # 2. Front Camera
    front_cam = Node(
        package='krsbi_vision',
        executable='camera_node',
        name='front_camera',
        namespace='krsbi',
        output='screen',
        parameters=[{
            'device_id': LaunchConfiguration('front_device'),
            'width': 640, 'height': 480, 'fps': 30
        }],
        remappings=[('image_raw', 'camera/front/image_raw')]
    )
    
    # 3. Ball Detector (Processes both)
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
            'publish_debug': True,
        }]
    )
    
    # 4. Ball Seeker Strategy
    ball_seeker = Node(
        package='krsbi_vision',
        executable='ball_seeker',
        name='ball_seeker',
        namespace='krsbi',
        output='screen',
        parameters=[{
            'target_distance': 0.2, # 20cm stop distance
            'dribble_threshold': 0.15
        }]
    )
    
    return LaunchDescription([
        omni_dev_arg, front_dev_arg, model_arg, class_arg,
        LogInfo(msg="Starting KRSBI Ball Seeker System..."),
        omni_cam,
        front_cam,
        ball_detector,
        ball_seeker
    ])
