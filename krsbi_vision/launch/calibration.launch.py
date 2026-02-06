#!/usr/bin/env python3
"""
KRSBI-B Soccer Robot - Calibration Launch File

Launches tools for camera and color calibration.

Usage:
    ros2 launch krsbi_vision calibration.launch.py
    ros2 launch krsbi_vision calibration.launch.py tool:=color
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    # Package directory
    pkg_vision = get_package_share_directory('krsbi_vision')
    
    # Config files
    camera_config = os.path.join(pkg_vision, 'config', 'camera_config.yaml')
    
    # =================================================================
    # Launch Arguments
    # =================================================================
    tool = LaunchConfiguration('tool')
    camera = LaunchConfiguration('camera')
    
    declare_tool = DeclareLaunchArgument(
        'tool',
        default_value='color',
        description='Calibration tool: color, camera'
    )
    
    declare_camera = DeclareLaunchArgument(
        'camera',
        default_value='front',
        description='Camera to calibrate: front, omni'
    )
    
    # =================================================================
    # Log info
    # =================================================================
    log_info = LogInfo(
        msg=['Starting Calibration Tool: ', tool, ' for camera: ', camera]
    )
    
    # =================================================================
    # Camera Node (for live feed)
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
            }
        ],
        condition=IfCondition(
            PythonExpression(["'", camera, "' == 'front'"])
        ),
    )
    
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
            }
        ],
        condition=IfCondition(
            PythonExpression(["'", camera, "' == 'omni'"])
        ),
    )
    
    # =================================================================
    # Color Calibrator Node
    # =================================================================
    color_calibrator_node = Node(
        package='krsbi_vision',
        executable='color_calibrator',
        name='color_calibrator',
        namespace='krsbi',
        output='screen',
        parameters=[
            {
                'camera_topic': PythonExpression([
                    "'/krsbi/camera/front/image_raw' if '", camera, 
                    "' == 'front' else '/krsbi/camera/omni/image_unwrapped'"
                ]),
                'config_path': os.path.join(pkg_vision, 'config', 'color_thresholds.yaml'),
            }
        ],
        condition=IfCondition(
            PythonExpression(["'", tool, "' == 'color'"])
        ),
    )
    
    # =================================================================
    # Launch Description
    # =================================================================
    return LaunchDescription([
        # Arguments
        declare_tool,
        declare_camera,
        
        # Log
        log_info,
        
        # Camera
        front_camera_node,
        omni_camera_node,
        
        # Tools
        color_calibrator_node,
    ])
