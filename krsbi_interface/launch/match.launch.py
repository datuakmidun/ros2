#!/usr/bin/env python3
"""
KRSBI-B Soccer Robot - Match Launch File

Launch file for competition/match mode.
Starts all nodes needed for autonomous operation.

Usage:
    ros2 launch krsbi_interface match.launch.py
    ros2 launch krsbi_interface match.launch.py robot_id:=1 team_color:=blue role:=striker
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, 
    IncludeLaunchDescription, 
    GroupAction,
    LogInfo,
)
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    # Package directories
    pkg_interface = get_package_share_directory('krsbi_interface')
    
    # Config files
    robot_params = os.path.join(pkg_interface, 'config', 'robot_params.yaml')
    vision_params = os.path.join(pkg_interface, 'config', 'vision_params.yaml')
    control_params = os.path.join(pkg_interface, 'config', 'control_params.yaml')
    game_params = os.path.join(pkg_interface, 'config', 'game_params.yaml')
    
    # Launch arguments
    robot_id = LaunchConfiguration('robot_id')
    team_color = LaunchConfiguration('team_color')
    role = LaunchConfiguration('role')
    
    # Declare launch arguments
    declare_robot_id = DeclareLaunchArgument(
        'robot_id',
        default_value='1',
        description='Robot ID (1-4)'
    )
    
    declare_team_color = DeclareLaunchArgument(
        'team_color',
        default_value='blue',
        description='Team color (blue or yellow)'
    )
    
    declare_role = DeclareLaunchArgument(
        'role',
        default_value='striker',
        description='Robot role (striker, goalkeeper, defender, support)'
    )
    
    # Log match configuration
    log_config = LogInfo(
        msg=['Starting KRSBI-B Match Mode - Robot ', robot_id, 
             ' | Team: ', team_color, ' | Role: ', role]
    )
    
    # Include robot bringup
    robot_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_interface, 'launch', 'robot_bringup.launch.py')
        ),
        launch_arguments={
            'robot_id': robot_id,
            'team_color': team_color,
            'role': role,
            'use_sim_time': 'false',
            'start_rviz': 'false',
        }.items()
    )
    
    # Note: Additional nodes would be added here when packages are ready
    # - krsbi_comm: Serial communication with Arduino
    # - krsbi_vision: Camera processing
    # - krsbi_control: Motion control
    # - krsbi_decision: Behavior/strategy
    
    # Placeholder for future nodes
    # comm_node = Node(
    #     package='krsbi_comm',
    #     executable='serial_node',
    #     name='serial_comm',
    #     output='screen',
    #     parameters=[robot_params],
    # )
    
    # vision_node = Node(
    #     package='krsbi_vision',
    #     executable='ball_detector',
    #     name='ball_detector',
    #     output='screen',
    #     parameters=[vision_params],
    # )
    
    # control_node = Node(
    #     package='krsbi_control',
    #     executable='motion_controller',
    #     name='motion_controller',
    #     output='screen',
    #     parameters=[control_params],
    # )
    
    # decision_node = Node(
    #     package='krsbi_decision',
    #     executable='behavior_manager',
    #     name='behavior_manager',
    #     output='screen',
    #     parameters=[game_params],
    # )
    
    return LaunchDescription([
        # Launch arguments
        declare_robot_id,
        declare_team_color,
        declare_role,
        
        # Log configuration
        log_config,
        
        # Robot bringup (includes param_server, system_monitor, etc.)
        robot_bringup,
        
        # Add additional match nodes here when ready
    ])
