#!/usr/bin/env python3
"""
KRSBI-B Robot - Robot State Publisher Launch File

Launches only the robot_state_publisher for use with other nodes.
This is typically included by other launch files.

Usage:
    ros2 launch krsbi_description robot_state_publisher.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():
    # Package directories
    pkg_description = get_package_share_directory('krsbi_description')
    
    # File paths
    urdf_file = os.path.join(pkg_description, 'urdf', 'robot.urdf.xacro')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    namespace = LaunchConfiguration('namespace')
    
    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    declare_namespace = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Robot namespace'
    )
    
    # Get URDF via xacro
    robot_description = Command(['xacro ', urdf_file])
    
    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=namespace,
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
            'publish_frequency': 50.0,
            'frame_prefix': '',
        }],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
        ]
    )
    
    return LaunchDescription([
        # Launch arguments
        declare_use_sim_time,
        declare_namespace,
        
        # Nodes
        robot_state_publisher,
    ])
