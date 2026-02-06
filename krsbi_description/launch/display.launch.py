#!/usr/bin/env python3
"""
KRSBI-B Robot - Display Launch File

Launches robot_state_publisher and RViz for visualization.
Use this for testing URDF and viewing robot model.

Usage:
    ros2 launch krsbi_description display.launch.py
    ros2 launch krsbi_description display.launch.py use_gui:=true
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():
    # Package directories
    pkg_description = get_package_share_directory('krsbi_description')
    
    # File paths
    urdf_file = os.path.join(pkg_description, 'urdf', 'robot.urdf.xacro')
    rviz_config = os.path.join(pkg_description, 'rviz', 'display.rviz')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_gui = LaunchConfiguration('use_gui')
    use_rviz = LaunchConfiguration('use_rviz')
    
    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    declare_use_gui = DeclareLaunchArgument(
        'use_gui',
        default_value='true',
        description='Use joint_state_publisher_gui for manual joint control'
    )
    
    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz visualization'
    )
    
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
            'use_sim_time': use_sim_time,
            'publish_frequency': 50.0,
        }]
    )
    
    # Joint State Publisher (without GUI)
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_gui', default='true').perform(None) == 'false' if False else 'true'),
        parameters=[{
            'use_sim_time': use_sim_time,
            'rate': 50,
        }]
    )
    
    # Joint State Publisher GUI (for testing)
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        condition=IfCondition(use_gui),
        parameters=[{
            'use_sim_time': use_sim_time,
        }]
    )
    
    # RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(use_rviz),
        arguments=['-d', rviz_config],
        parameters=[{
            'use_sim_time': use_sim_time,
        }]
    )
    
    return LaunchDescription([
        # Launch arguments
        declare_use_sim_time,
        declare_use_gui,
        declare_use_rviz,
        
        # Nodes
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz,
    ])
