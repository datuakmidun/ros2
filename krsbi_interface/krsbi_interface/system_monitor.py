#!/usr/bin/env python3
"""
KRSBI-B Soccer Robot - System Monitor Node

Monitors system health, battery status, and publishes diagnostics.

Publishes:
    - /diagnostics (diagnostic_msgs/DiagnosticArray)
    - /krsbi/system_status (std_msgs/String)

Subscribes:
    - /krsbi/robot_state (krsbi_msgs/RobotState)
    - /krsbi/sensor_data (krsbi_msgs/SensorData)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

import psutil
import time
from typing import Dict, Any

# Try to import krsbi_msgs
try:
    from krsbi_msgs.msg import RobotState, SensorData
    MSGS_AVAILABLE = True
except ImportError:
    MSGS_AVAILABLE = False


class SystemMonitor(Node):
    """
    System health monitoring node.
    
    Monitors:
    - CPU usage and temperature
    - Memory usage
    - Battery status
    - Communication status
    - Sensor health
    """
    
    def __init__(self):
        super().__init__('system_monitor')
        
        # Parameters
        self.declare_parameter('publish_rate', 1.0)
        self.declare_parameter('battery_low_threshold', 14.0)
        self.declare_parameter('battery_critical_threshold', 13.2)
        self.declare_parameter('cpu_warning_threshold', 80.0)
        self.declare_parameter('memory_warning_threshold', 80.0)
        self.declare_parameter('comm_timeout', 1.0)
        
        self.publish_rate = self.get_parameter('publish_rate').value
        self.battery_low = self.get_parameter('battery_low_threshold').value
        self.battery_critical = self.get_parameter('battery_critical_threshold').value
        self.cpu_warning = self.get_parameter('cpu_warning_threshold').value
        self.memory_warning = self.get_parameter('memory_warning_threshold').value
        self.comm_timeout = self.get_parameter('comm_timeout').value
        
        # State tracking
        self.last_robot_state_time = None
        self.last_sensor_data_time = None
        self.robot_state = None
        self.sensor_data = None
        
        # Publishers
        self.diagnostics_pub = self.create_publisher(
            DiagnosticArray,
            '/diagnostics',
            10
        )
        
        self.status_pub = self.create_publisher(
            String,
            '/krsbi/system_status',
            10
        )
        
        # Subscribers (if messages available)
        if MSGS_AVAILABLE:
            self.robot_state_sub = self.create_subscription(
                RobotState,
                '/krsbi/robot_state',
                self.robot_state_callback,
                10
            )
            
            self.sensor_data_sub = self.create_subscription(
                SensorData,
                '/krsbi/sensor_data',
                self.sensor_data_callback,
                10
            )
        
        # Timer
        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.publish_diagnostics)
        
        self.get_logger().info('System Monitor started')
    
    def robot_state_callback(self, msg: 'RobotState'):
        """Handle robot state updates."""
        self.robot_state = msg
        self.last_robot_state_time = time.time()
    
    def sensor_data_callback(self, msg: 'SensorData'):
        """Handle sensor data updates."""
        self.sensor_data = msg
        self.last_sensor_data_time = time.time()
    
    def get_cpu_status(self) -> DiagnosticStatus:
        """Get CPU diagnostic status."""
        status = DiagnosticStatus()
        status.name = "CPU"
        status.hardware_id = "Intel NUC"
        
        try:
            cpu_percent = psutil.cpu_percent(interval=0.1)
            cpu_freq = psutil.cpu_freq()
            
            status.values = [
                KeyValue(key="Usage", value=f"{cpu_percent:.1f}%"),
                KeyValue(key="Frequency", value=f"{cpu_freq.current:.0f} MHz" if cpu_freq else "N/A"),
                KeyValue(key="Cores", value=str(psutil.cpu_count())),
            ]
            
            # Try to get temperature (Linux only)
            try:
                temps = psutil.sensors_temperatures()
                if temps:
                    for name, entries in temps.items():
                        if entries:
                            temp = entries[0].current
                            status.values.append(
                                KeyValue(key="Temperature", value=f"{temp:.1f}°C")
                            )
                            break
            except Exception:
                pass
            
            if cpu_percent > self.cpu_warning:
                status.level = DiagnosticStatus.WARN
                status.message = f"High CPU usage: {cpu_percent:.1f}%"
            else:
                status.level = DiagnosticStatus.OK
                status.message = f"CPU usage: {cpu_percent:.1f}%"
                
        except Exception as e:
            status.level = DiagnosticStatus.ERROR
            status.message = f"Error reading CPU: {e}"
        
        return status
    
    def get_memory_status(self) -> DiagnosticStatus:
        """Get memory diagnostic status."""
        status = DiagnosticStatus()
        status.name = "Memory"
        status.hardware_id = "System RAM"
        
        try:
            mem = psutil.virtual_memory()
            
            status.values = [
                KeyValue(key="Total", value=f"{mem.total / (1024**3):.1f} GB"),
                KeyValue(key="Used", value=f"{mem.used / (1024**3):.1f} GB"),
                KeyValue(key="Available", value=f"{mem.available / (1024**3):.1f} GB"),
                KeyValue(key="Percent", value=f"{mem.percent:.1f}%"),
            ]
            
            if mem.percent > self.memory_warning:
                status.level = DiagnosticStatus.WARN
                status.message = f"High memory usage: {mem.percent:.1f}%"
            else:
                status.level = DiagnosticStatus.OK
                status.message = f"Memory usage: {mem.percent:.1f}%"
                
        except Exception as e:
            status.level = DiagnosticStatus.ERROR
            status.message = f"Error reading memory: {e}"
        
        return status
    
    def get_battery_status(self) -> DiagnosticStatus:
        """Get battery diagnostic status."""
        status = DiagnosticStatus()
        status.name = "Battery"
        status.hardware_id = "LiPo 4S"
        
        if self.robot_state is not None:
            voltage = self.robot_state.battery_voltage
            percentage = self.robot_state.battery_percentage
            
            status.values = [
                KeyValue(key="Voltage", value=f"{voltage:.2f} V"),
                KeyValue(key="Percentage", value=f"{percentage:.1f}%"),
                KeyValue(key="Low Warning", value=str(self.robot_state.battery_low)),
            ]
            
            if voltage < self.battery_critical:
                status.level = DiagnosticStatus.ERROR
                status.message = f"CRITICAL: Battery at {voltage:.2f}V"
            elif voltage < self.battery_low:
                status.level = DiagnosticStatus.WARN
                status.message = f"Low battery: {voltage:.2f}V"
            else:
                status.level = DiagnosticStatus.OK
                status.message = f"Battery OK: {voltage:.2f}V ({percentage:.0f}%)"
        else:
            status.level = DiagnosticStatus.STALE
            status.message = "No battery data"
            status.values = []
        
        return status
    
    def get_communication_status(self) -> DiagnosticStatus:
        """Get communication diagnostic status."""
        status = DiagnosticStatus()
        status.name = "Communication"
        status.hardware_id = "Serial/Arduino"
        
        current_time = time.time()
        
        # Check robot state communication
        robot_state_ok = (
            self.last_robot_state_time is not None and
            current_time - self.last_robot_state_time < self.comm_timeout
        )
        
        # Check sensor data communication
        sensor_data_ok = (
            self.last_sensor_data_time is not None and
            current_time - self.last_sensor_data_time < self.comm_timeout
        )
        
        status.values = [
            KeyValue(key="Robot State", value="OK" if robot_state_ok else "TIMEOUT"),
            KeyValue(key="Sensor Data", value="OK" if sensor_data_ok else "TIMEOUT"),
        ]
        
        if self.robot_state is not None:
            status.values.append(
                KeyValue(key="Communication OK", 
                        value=str(self.robot_state.communication_ok))
            )
        
        if robot_state_ok and sensor_data_ok:
            status.level = DiagnosticStatus.OK
            status.message = "Communication OK"
        elif robot_state_ok or sensor_data_ok:
            status.level = DiagnosticStatus.WARN
            status.message = "Partial communication"
        else:
            status.level = DiagnosticStatus.ERROR
            status.message = "Communication lost"
        
        return status
    
    def get_sensors_status(self) -> DiagnosticStatus:
        """Get sensors diagnostic status."""
        status = DiagnosticStatus()
        status.name = "Sensors"
        status.hardware_id = "Arduino Mega"
        
        if self.robot_state is not None:
            motors_ok = self.robot_state.motors_ok
            sensors_ok = self.robot_state.sensors_ok
            
            status.values = [
                KeyValue(key="Motors", value="OK" if motors_ok else "ERROR"),
                KeyValue(key="Sensors", value="OK" if sensors_ok else "ERROR"),
                KeyValue(key="Localized", value=str(self.robot_state.is_localized)),
                KeyValue(key="Has Ball", value=str(self.robot_state.has_ball)),
            ]
            
            if motors_ok and sensors_ok:
                status.level = DiagnosticStatus.OK
                status.message = "All sensors OK"
            else:
                status.level = DiagnosticStatus.WARN
                status.message = "Sensor issues detected"
                if self.robot_state.error_message:
                    status.values.append(
                        KeyValue(key="Error", value=self.robot_state.error_message)
                    )
        else:
            status.level = DiagnosticStatus.STALE
            status.message = "No sensor data"
            status.values = []
        
        return status
    
    def publish_diagnostics(self):
        """Publish diagnostic array."""
        diag_array = DiagnosticArray()
        diag_array.header.stamp = self.get_clock().now().to_msg()
        
        # Collect all diagnostics
        diag_array.status.append(self.get_cpu_status())
        diag_array.status.append(self.get_memory_status())
        diag_array.status.append(self.get_battery_status())
        diag_array.status.append(self.get_communication_status())
        diag_array.status.append(self.get_sensors_status())
        
        self.diagnostics_pub.publish(diag_array)
        
        # Publish simple status summary
        overall_status = "OK"
        for status in diag_array.status:
            if status.level == DiagnosticStatus.ERROR:
                overall_status = "ERROR"
                break
            elif status.level == DiagnosticStatus.WARN:
                overall_status = "WARNING"
        
        status_msg = String()
        status_msg.data = overall_status
        self.status_pub.publish(status_msg)


def main(args=None):
    rclpy.init(args=args)
    
    node = SystemMonitor()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
