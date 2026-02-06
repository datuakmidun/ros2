#!/usr/bin/env python3
"""
KRSBI-B Soccer Robot - Robot Bringup Launch File

Main launch file that starts all robot components.

Usage:
    ros2 launch krsbi_interface robot_bringup.launch.py
    ros2 launch krsbi_interface robot_bringup.launch.py robot_id:=1 team_color:=blue
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    # Package directories
    pkg_interface = get_package_share_directory('krsbi_interface')
    
    # Try to get other packages (may not exist yet)
    try:
        pkg_description = get_package_share_directory('krsbi_description')
    except Exception:
        pkg_description = None
    
    # Config files
    robot_params = os.path.join(pkg_interface, 'config', 'robot_params.yaml')
    vision_params = os.path.join(pkg_interface, 'config', 'vision_params.yaml')
    control_params = os.path.join(pkg_interface, 'config', 'control_params.yaml')
    game_params = os.path.join(pkg_interface, 'config', 'game_params.yaml')
    
    # Launch arguments
    robot_id = LaunchConfiguration('robot_id')
    team_color = LaunchConfiguration('team_color')
    role = LaunchConfiguration('role')
    use_sim_time = LaunchConfiguration('use_sim_time')
    start_rviz = LaunchConfiguration('start_rviz')
    
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
    
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    declare_start_rviz = DeclareLaunchArgument(
        'start_rviz',
        default_value='false',
        description='Start RViz visualization'
    )
    
    # Parameter server
    param_server = Node(
        package='krsbi_interface',
        executable='param_server',
        name='param_server',
        output='screen',
        parameters=[
            robot_params,
            {'use_sim_time': use_sim_time},
        ]
    )
    
    # System monitor
    system_monitor = Node(
        package='krsbi_interface',
        executable='system_monitor',
        name='system_monitor',
        output='screen',
        parameters=[
            {'publish_rate': 1.0},
            {'battery_low_threshold': 14.0},
            {'battery_critical_threshold': 13.2},
            {'use_sim_time': use_sim_time},
        ]
    )
    
    # Robot state publisher (from krsbi_description)
    robot_state_publisher = None
    if pkg_description:
        robot_state_publisher = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_description, 'launch', 'robot_state_publisher.launch.py')
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
            }.items()
        )
    
    # Build launch description
    ld = LaunchDescription()
    
    # Add launch arguments
    ld.add_action(declare_robot_id)
    ld.add_action(declare_team_color)
    ld.add_action(declare_role)
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_start_rviz)
    
    # Add nodes in namespace
    ld.add_action(GroupAction([
        PushRosNamespace('krsbi'),
        param_server,
        system_monitor,
    ]))
    
    # Add robot state publisher if available
    if robot_state_publisher:
        ld.add_action(robot_state_publisher)
    
    return ld
