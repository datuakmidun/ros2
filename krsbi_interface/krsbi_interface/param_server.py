#!/usr/bin/env python3
"""
KRSBI-B Soccer Robot - Parameter Server Node

Loads and provides centralized access to all robot parameters.

Services:
    - /krsbi/get_param (rcl_interfaces/srv/GetParameters)
    - /krsbi/set_param (rcl_interfaces/srv/SetParameters)

Parameters are loaded from config files and can be accessed by all nodes.
"""

import os
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from ament_index_python.packages import get_package_share_directory

import yaml
from typing import Dict, Any


class ParamServer(Node):
    """
    Centralized parameter server for KRSBI-B robot.
    
    Loads parameters from YAML files and declares them as ROS 2 parameters.
    """
    
    def __init__(self):
        super().__init__('param_server')
        
        # Get package share directory
        try:
            self.config_dir = os.path.join(
                get_package_share_directory('krsbi_interface'),
                'config'
            )
        except Exception:
            # Fallback for development
            self.config_dir = os.path.join(
                os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                'config'
            )
        
        # Load all config files
        self.load_config_file('robot_params.yaml', 'robot')
        self.load_config_file('vision_params.yaml', 'vision')
        self.load_config_file('control_params.yaml', 'control')
        self.load_config_file('game_params.yaml', 'game')
        
        self.get_logger().info(f'Parameter server started with config from: {self.config_dir}')
    
    def load_config_file(self, filename: str, namespace: str):
        """
        Load parameters from a YAML config file.
        
        Args:
            filename: Name of the YAML file
            namespace: Namespace prefix for parameters
        """
        filepath = os.path.join(self.config_dir, filename)
        
        if not os.path.exists(filepath):
            self.get_logger().warning(f'Config file not found: {filepath}')
            return
        
        try:
            with open(filepath, 'r') as f:
                config = yaml.safe_load(f)
            
            if config:
                self.declare_nested_params(config, namespace)
                self.get_logger().info(f'Loaded {filename}')
                
        except Exception as e:
            self.get_logger().error(f'Error loading {filename}: {e}')
    
    def declare_nested_params(self, data: Dict[str, Any], prefix: str = ''):
        """
        Recursively declare nested parameters.
        
        Args:
            data: Dictionary of parameters
            prefix: Current parameter prefix
        """
        for key, value in data.items():
            param_name = f'{prefix}.{key}' if prefix else key
            
            if isinstance(value, dict):
                # Recurse into nested dictionaries
                self.declare_nested_params(value, param_name)
            else:
                # Declare the parameter
                self.declare_param(param_name, value)
    
    def declare_param(self, name: str, value: Any):
        """
        Declare a single parameter with appropriate type.
        
        Args:
            name: Parameter name
            value: Parameter value
        """
        try:
            # Determine parameter type
            if isinstance(value, bool):
                param_type = ParameterType.PARAMETER_BOOL
            elif isinstance(value, int):
                param_type = ParameterType.PARAMETER_INTEGER
            elif isinstance(value, float):
                param_type = ParameterType.PARAMETER_DOUBLE
            elif isinstance(value, str):
                param_type = ParameterType.PARAMETER_STRING
            elif isinstance(value, list):
                if all(isinstance(x, int) for x in value):
                    param_type = ParameterType.PARAMETER_INTEGER_ARRAY
                elif all(isinstance(x, (int, float)) for x in value):
                    param_type = ParameterType.PARAMETER_DOUBLE_ARRAY
                elif all(isinstance(x, str) for x in value):
                    param_type = ParameterType.PARAMETER_STRING_ARRAY
                else:
                    # Convert to string array
                    value = [str(x) for x in value]
                    param_type = ParameterType.PARAMETER_STRING_ARRAY
            else:
                # Convert to string
                value = str(value)
                param_type = ParameterType.PARAMETER_STRING
            
            descriptor = ParameterDescriptor()
            descriptor.type = param_type
            descriptor.read_only = False
            
            self.declare_parameter(name, value, descriptor)
            
        except Exception as e:
            self.get_logger().debug(f'Could not declare {name}: {e}')
    
    def get_all_params(self) -> Dict[str, Any]:
        """Get all declared parameters as a dictionary."""
        params = {}
        for param in self._parameters.values():
            params[param.name] = param.value
        return params


def main(args=None):
    rclpy.init(args=args)
    
    node = ParamServer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
