#!/usr/bin/env python3
"""
KRSBI-B Soccer Robot - Control Bringup

Starts:
- Localization (Odometry)
- Motion Controller (Ramping/Limits)
- Path Planner (Simple Obstacle Avoidance)
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_control = get_package_share_directory('krsbi_control')
    
    # Config
    control_config = os.path.join(pkg_control, 'config', 'control_config.yaml')
    
    # Arguments
    use_imu = LaunchConfiguration('use_imu')
    declare_use_imu = DeclareLaunchArgument(
        'use_imu',
        default_value='true',
        description='Use IMU for localization'
    )
    
    # Localization
    localization_node = Node(
        package='krsbi_control',
        executable='localization',
        name='localization',
        output='screen',
        parameters=[
            control_config,
            {'use_imu': use_imu}
        ]
    )
    
    # Motion Controller
    motion_controller_node = Node(
        package='krsbi_control',
        executable='motion_controller',
        name='motion_controller',
        output='screen',
        parameters=[control_config]
    )
    
    # Path Planner
    path_planner_node = Node(
        package='krsbi_control',
        executable='path_planner',
        name='path_planner',
        output='screen',
        parameters=[control_config]
    )
    
    return LaunchDescription([
        declare_use_imu,
        LogInfo(msg='Starting KRSBI Control System...'),
        localization_node,
        motion_controller_node,
        path_planner_node
    ])
