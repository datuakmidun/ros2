#!/usr/bin/env python3
"""
KRSBI-B Soccer Robot - Communication Bringup Launch File

Launches serial communication node with Arduino.

Usage:
    ros2 launch krsbi_comm comm_bringup.launch.py
    ros2 launch krsbi_comm comm_bringup.launch.py port:=/dev/ttyUSB0
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Package directories
    pkg_comm = get_package_share_directory('krsbi_comm')
    
    # Config files
    serial_config = os.path.join(pkg_comm, 'config', 'serial_config.yaml')
    
    # Launch arguments
    port = LaunchConfiguration('port')
    baudrate = LaunchConfiguration('baudrate')
    debug = LaunchConfiguration('debug')
    
    # Declare launch arguments
    declare_port = DeclareLaunchArgument(
        'port',
        default_value='/dev/ttyUSB0',
        description='Serial port for Arduino (Linux: /dev/ttyUSB0, Windows: COM3)'
    )
    
    declare_baudrate = DeclareLaunchArgument(
        'baudrate',
        default_value='115200',
        description='Serial baud rate'
    )
    
    declare_debug = DeclareLaunchArgument(
        'debug',
        default_value='false',
        description='Enable debug logging'
    )
    
    # Log configuration
    log_config = LogInfo(
        msg=['Starting Serial Comm - Port: ', port, ' Baudrate: ', baudrate]
    )
    
    # Serial communication node
    serial_node = Node(
        package='krsbi_comm',
        executable='serial_node',
        name='serial_comm_node',
        namespace='krsbi',
        output='screen',
        parameters=[
            serial_config,
            {
                'port': port,
                'baudrate': baudrate,
                'debug': debug,
            }
        ],
        remappings=[
            ('motor_command', '/krsbi/motor_command'),
            ('cmd_vel', '/krsbi/cmd_vel'),
            ('sensor_data', '/krsbi/sensor_data'),
            ('imu', '/krsbi/imu'),
            ('motor_feedback', '/krsbi/motor_feedback'),
            ('distance_sensors', '/krsbi/distance_sensors'),
        ]
    )
    
    return LaunchDescription([
        # Launch arguments
        declare_port,
        declare_baudrate,
        declare_debug,
        
        # Log
        log_config,
        
        # Nodes
        serial_node,
    ])
