#!/usr/bin/env python3
"""
KRSBI-B Soccer Robot - Vision Bringup Launch File

Launches all vision nodes for dual camera setup.

Usage:
    ros2 launch krsbi_vision vision_bringup.launch.py
    ros2 launch krsbi_vision vision_bringup.launch.py use_yolo:=true
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, LogInfo
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    # Package directory
    pkg_vision = get_package_share_directory('krsbi_vision')
    
    # Config files
    camera_config = os.path.join(pkg_vision, 'config', 'camera_config.yaml')
    detection_config = os.path.join(pkg_vision, 'config', 'detection_config.yaml')
    tracking_config = os.path.join(pkg_vision, 'config', 'tracking_config.yaml')
    
    # =================================================================
    # Launch Arguments
    # =================================================================
    use_front_camera = LaunchConfiguration('use_front_camera')
    use_omni_camera = LaunchConfiguration('use_omni_camera')
    use_yolo = LaunchConfiguration('use_yolo')
    use_color = LaunchConfiguration('use_color')
    publish_debug = LaunchConfiguration('publish_debug')
    
    declare_front_camera = DeclareLaunchArgument(
        'use_front_camera',
        default_value='true',
        description='Enable front camera'
    )
    
    declare_omni_camera = DeclareLaunchArgument(
        'use_omni_camera',
        default_value='true',
        description='Enable omni camera'
    )
    
    declare_use_yolo = DeclareLaunchArgument(
        'use_yolo',
        default_value='true',
        description='Use YOLOv8 for detection'
    )
    
    declare_use_color = DeclareLaunchArgument(
        'use_color',
        default_value='true',
        description='Use color-based detection'
    )
    
    declare_publish_debug = DeclareLaunchArgument(
        'publish_debug',
        default_value='true',
        description='Publish debug images'
    )
    
    # =================================================================
    # Log info
    # =================================================================
    log_info = LogInfo(
        msg=['Starting Vision System - YOLO: ', use_yolo, ', Color: ', use_color]
    )
    
    # =================================================================
    # Front Camera Node
    # =================================================================
    front_camera_node = Node(
        package='krsbi_vision',
        executable='camera_node',
        name='front_camera',
        namespace='krsbi',
        output='screen',
        parameters=[
            camera_config,
            {
                'device_id': 1,
                'frame_id': 'front_camera_link',
            }
        ],
        condition=IfCondition(use_front_camera),
    )
    
    # =================================================================
    # Omni Camera Node
    # =================================================================
    omni_camera_node = Node(
        package='krsbi_vision',
        executable='omni_camera_node',
        name='omni_camera',
        namespace='krsbi',
        output='screen',
        parameters=[
            camera_config,
            {
                'device_id': 0,
                'frame_id': 'omni_camera_link',
            }
        ],
        condition=IfCondition(use_omni_camera),
    )
    
    # =================================================================
    # Ball Detector Node
    # =================================================================
    ball_detector_node = Node(
        package='krsbi_vision',
        executable='ball_detector',
        name='ball_detector',
        namespace='krsbi',
        output='screen',
        parameters=[
            detection_config,
            tracking_config,
            {
                'use_yolo': use_yolo,
                'use_color': use_color,
                'publish_debug': publish_debug,
            }
        ],
    )
    
    # =================================================================
    # Ball Tracker Node
    # =================================================================
    ball_tracker_node = Node(
        package='krsbi_vision',
        executable='ball_tracker',
        name='ball_tracker',
        namespace='krsbi',
        output='screen',
        parameters=[
            tracking_config,
        ],
    )
    
    # =================================================================
    # Robot Detector Node
    # =================================================================
    robot_detector_node = Node(
        package='krsbi_vision',
        executable='robot_detector',
        name='robot_detector',
        namespace='krsbi',
        output='screen',
        parameters=[
            detection_config,
            {
                'use_yolo': use_yolo,
                'publish_debug': publish_debug,
            }
        ],
    )
    
    # =================================================================
    # Field Detector Node
    # =================================================================
    field_detector_node = Node(
        package='krsbi_vision',
        executable='field_detector',
        name='field_detector',
        namespace='krsbi',
        output='screen',
        parameters=[
            detection_config,
            {
                'publish_debug': publish_debug,
            }
        ],
    )
    
    # =================================================================
    # Vision Fusion Node
    # =================================================================
    vision_fusion_node = Node(
        package='krsbi_vision',
        executable='vision_fusion',
        name='vision_fusion',
        namespace='krsbi',
        output='screen',
        parameters=[
            camera_config,
        ],
    )
    
    # =================================================================
    # Launch Description
    # =================================================================
    return LaunchDescription([
        # Arguments
        declare_front_camera,
        declare_omni_camera,
        declare_use_yolo,
        declare_use_color,
        declare_publish_debug,
        
        # Log
        log_info,
        
        # Camera Nodes
        front_camera_node,
        omni_camera_node,
        
        # Detection Nodes
        ball_detector_node,
        ball_tracker_node,
        robot_detector_node,
        field_detector_node,
        
        # Fusion
        vision_fusion_node,
    ])
