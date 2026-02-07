#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get package share directory
    pkg_share = get_package_share_directory('krsbi_vision')
    
    # Default model path (relative to package)
    default_model = os.path.join(pkg_share, 'models', 'yolov8s.pt')
    
    # Launch arguments
    device_arg = DeclareLaunchArgument('device', default_value='0', description='Camera device ID')
    model_arg = DeclareLaunchArgument('model', default_value=default_model, description='YOLO model path')
    class_arg = DeclareLaunchArgument('ball_class', default_value='0', description='Ball class ID')
    
    camera_node = Node(
        package='krsbi_vision',
        executable='camera_node',
        name='camera',
        namespace='krsbi',
        output='screen',
        parameters=[{'device_id': LaunchConfiguration('device'), 'width': 640, 'height': 480, 'fps': 30}],
        remappings=[('/krsbi/camera/image_raw', '/krsbi/camera/front/image_raw')],
    )
    
    ball_detector = Node(
        package='krsbi_vision',
        executable='ball_detector',
        name='ball_detector',
        namespace='krsbi',
        output='screen',
        parameters=[{
            'use_yolo': True,
            'yolo_model': LaunchConfiguration('model'),
            'ball_class_id': LaunchConfiguration('ball_class'),
            'yolo_confidence': 0.5,
            'publish_debug': True,
            'detection_rate': 20.0
        }]
    )
    
    return LaunchDescription([device_arg, model_arg, class_arg, camera_node, ball_detector])

