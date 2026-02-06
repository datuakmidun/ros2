#!/usr/bin/env python3
"""
KRSBI-B Robot - View Frames Launch File

Launches robot state publisher and frames viewer for debugging TF tree.

Usage:
    ros2 launch krsbi_description view_frames.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():
    # Package directories
    pkg_description = get_package_share_directory('krsbi_description')
    
    # File paths
    urdf_file = os.path.join(pkg_description, 'urdf', 'robot.urdf.xacro')
    
    # Get URDF via xacro
    robot_description = Command(['xacro ', urdf_file])
    
    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'publish_frequency': 50.0,
        }]
    )
    
    # Joint State Publisher (minimal)
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{
            'rate': 50,
        }]
    )
    
    # View TF frames (generates PDF)
    view_frames = ExecuteProcess(
        cmd=['ros2', 'run', 'tf2_tools', 'view_frames'],
        output='screen',
    )
    
    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher,
        # Uncomment to auto-generate frames.pdf:
        # view_frames,
    ])
