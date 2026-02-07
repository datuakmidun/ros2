#!/usr/bin/env python3
"""
KRSBI-B Soccer Robot - Decision Bringup

Starts:
- Game Controller (Referee Interface)
- Strategy Manager (Behavior Tree Execution)
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_decision = get_package_share_directory('krsbi_decision')
    
    # Config
    strategy_params = os.path.join(pkg_decision, 'config', 'strategy_params.yaml')
    
    # Arguments
    role = LaunchConfiguration('role')
    declare_role = DeclareLaunchArgument(
        'role',
        default_value='striker',
        description='Role of the robot: striker, goalie, defender'
    )
    
    # Game Controller
    game_controller_node = Node(
        package='krsbi_decision',
        executable='game_controller',
        name='game_controller',
        output='screen'
    )
    
    # Strategy Manager
    strategy_manager_node = Node(
        package='krsbi_decision',
        executable='strategy_manager',
        name='strategy_manager',
        output='screen',
        parameters=[
            {'role': role},
            strategy_params
        ]
    )
    
    return LaunchDescription([
        declare_role,
        LogInfo(msg='Starting KRSBI Decision System...'),
        game_controller_node,
        strategy_manager_node
    ])
