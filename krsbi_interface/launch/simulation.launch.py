#!/usr/bin/env python3
"""
KRSBI-B Soccer Robot - Simulation Launch File

Launch file for simulation/testing mode.
Uses simulation time and optionally starts Gazebo.

Usage:
    ros2 launch krsbi_interface simulation.launch.py
    ros2 launch krsbi_interface simulation.launch.py gui:=true
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, 
    IncludeLaunchDescription,
    ExecuteProcess,
    GroupAction,
    LogInfo,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    # Package directories
    pkg_interface = get_package_share_directory('krsbi_interface')
    
    # Try to get other packages
    try:
        pkg_description = get_package_share_directory('krsbi_description')
    except Exception:
        pkg_description = None
    
    # Config files
    robot_params = os.path.join(pkg_interface, 'config', 'robot_params.yaml')
    
    # Launch arguments
    use_gui = LaunchConfiguration('gui')
    world_file = LaunchConfiguration('world')
    robot_id = LaunchConfiguration('robot_id')
    team_color = LaunchConfiguration('team_color')
    
    # Declare launch arguments
    declare_use_gui = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Start Gazebo with GUI'
    )
    
    declare_world = DeclareLaunchArgument(
        'world',
        default_value='soccer_field',
        description='World file name (without .sdf)'
    )
    
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
    
    # Log simulation start
    log_info = LogInfo(
        msg=['Starting KRSBI-B Simulation Mode']
    )
    
    # Include robot bringup with simulation time
    robot_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_interface, 'launch', 'robot_bringup.launch.py')
        ),
        launch_arguments={
            'robot_id': robot_id,
            'team_color': team_color,
            'use_sim_time': 'true',
            'start_rviz': 'true',
        }.items()
    )
    
    # Note: Gazebo launch would be added here
    # This requires gazebo_ros package and world files
    
    # gazebo = ExecuteProcess(
    #     cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so',
    #          '-s', 'libgazebo_ros_factory.so', world_file],
    #     output='screen',
    # )
    
    # spawn_robot = Node(
    #     package='gazebo_ros',
    #     executable='spawn_entity.py',
    #     arguments=[
    #         '-topic', 'robot_description',
    #         '-entity', 'krsbi_robot',
    #         '-x', '0', '-y', '0', '-z', '0.1',
    #     ],
    #     output='screen',
    # )
    
    return LaunchDescription([
        # Launch arguments
        declare_use_gui,
        declare_world,
        declare_robot_id,
        declare_team_color,
        
        # Log info
        log_info,
        
        # Robot bringup with simulation time
        robot_bringup,
        
        # Add Gazebo nodes here when ready
        # gazebo,
        # spawn_robot,
    ])
