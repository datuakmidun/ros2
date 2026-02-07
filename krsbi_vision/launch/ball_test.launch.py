#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    device_arg = DeclareLaunchArgument('device', default_value='0', description='Camera device ID')
    
    camera_node = Node(
        package='krsbi_vision',
        executable='camera_node',
        name='camera',
        namespace='krsbi',
        output='screen',
        parameters=[{'device_id': LaunchConfiguration('device'), 'width': 640, 'height': 480, 'fps': 30.0}],
        remappings=[('/krsbi/camera/image_raw', '/krsbi/camera/front/image_raw')],
    )
    
    ball_detector = Node(
        package='krsbi_vision',
        executable='ball_detector',
        name='ball_detector',
        namespace='krsbi',
        output='screen',
        parameters=[{'use_yolo': True, 'yolo_model': 'yolov8n.pt', 'publish_debug': True, 'detection_rate': 20.0}]
    )
    
    return LaunchDescription([device_arg, camera_node, ball_detector])
