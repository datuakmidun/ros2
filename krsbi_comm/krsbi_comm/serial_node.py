#!/usr/bin/env python3
"""
KRSBI-B Soccer Robot - Serial Communication Node

Main ROS 2 node for bidirectional communication with Arduino Mega.

Subscribes to:
    - /krsbi/motor_command (krsbi_msgs/MotorCommand)
    - /krsbi/cmd_vel (geometry_msgs/Twist)

Publishes:
    - /krsbi/sensor_data (krsbi_msgs/SensorData)
    - /krsbi/imu (krsbi_msgs/ImuData)
    - /krsbi/motor_feedback (krsbi_msgs/MotorFeedback)
    - /krsbi/distance_sensors (krsbi_msgs/DistanceSensors)
    - /krsbi/robot_state (krsbi_msgs/RobotState)

Services:
    - /krsbi/set_gripper (krsbi_msgs/SetGripper)
    - /krsbi/charge_kicker (krsbi_msgs/ChargeKicker)
    - /krsbi/kick (krsbi_msgs/Kick)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue

import serial
import serial.tools.list_ports
import threading
import time
import queue
from typing import Optional, List

from .protocol import (
    Command, Response, Packet, PacketParser,
    build_motor_velocity, build_motor_rpm, build_motor_pwm, build_motor_stop,
    build_kicker_charge, build_kicker_kick, build_kicker_stop,
    build_gripper_open, build_gripper_close, build_gripper_position,
    build_request_sensors, build_heartbeat, build_emergency_stop,
    parse_sensor_all, parse_imu, parse_motor_feedback, parse_distance,
    SensorAllData, ImuData, MotorFeedbackData, DistanceData,
)

# Try to import krsbi_msgs
try:
    from krsbi_msgs.msg import (
        MotorCommand, MotorFeedback as MotorFeedbackMsg,
        SensorData, ImuData as ImuDataMsg,
        DistanceSensors, GripperState, KickerState, RobotState,
    )
    from krsbi_msgs.srv import SetGripper, ChargeKicker, Kick
    MSGS_AVAILABLE = True
except ImportError:
    MSGS_AVAILABLE = False
    print("Warning: krsbi_msgs not found. Running in limited mode.")


class SerialCommNode(Node):
    """
    Serial communication node for Arduino Mega.
    
    Handles:
    - Serial port connection and reconnection
    - Bidirectional packet communication
    - Sensor data publishing
    - Motor command sending
    - Gripper and kicker control
    """
    
    def __init__(self):
        super().__init__('serial_comm_node')
        
        # =================================================================
        # Parameters
        # =================================================================
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('timeout', 0.1)
        self.declare_parameter('auto_reconnect', True)
        self.declare_parameter('reconnect_interval', 2.0)
        self.declare_parameter('heartbeat_interval', 0.5)
        self.declare_parameter('heartbeat_timeout', 2.0)
        self.declare_parameter('sensor_rate', 50.0)
        self.declare_parameter('debug', False)
        
        self.port = self.get_parameter('port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.timeout = self.get_parameter('timeout').value
        self.auto_reconnect = self.get_parameter('auto_reconnect').value
        self.reconnect_interval = self.get_parameter('reconnect_interval').value
        self.heartbeat_interval = self.get_parameter('heartbeat_interval').value
        self.heartbeat_timeout = self.get_parameter('heartbeat_timeout').value
        self.sensor_rate = self.get_parameter('sensor_rate').value
        self.debug = self.get_parameter('debug').value
        
        # =================================================================
        # State Variables
        # =================================================================
        self.serial: Optional[serial.Serial] = None
        self.is_connected = False
        self.is_running = True
        self.last_heartbeat_time = time.time()
        self.last_sensor_time = time.time()
        self.emergency_stop_active = False
        
        # Statistics
        self.packets_sent = 0
        self.packets_received = 0
        self.packets_failed = 0
        self.crc_errors = 0
        
        # Threading
        self.write_queue = queue.Queue(maxsize=20)
        self.parser = PacketParser()
        self.lock = threading.Lock()
        
        # Last sensor data (for state publishing)
        self.last_sensor_data: Optional[SensorAllData] = None
        
        # =================================================================
        # QoS Profile
        # =================================================================
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # =================================================================
        # Publishers
        # =================================================================
        self.connection_pub = self.create_publisher(
            Bool, '/krsbi/comm/connected', 10)
        
        self.status_pub = self.create_publisher(
            String, '/krsbi/comm/status', 10)
        
        # Sensor publishers (only if msgs available)
        if MSGS_AVAILABLE:
            self.sensor_pub = self.create_publisher(
                SensorData, '/krsbi/sensor_data', sensor_qos)
            
            self.imu_pub = self.create_publisher(
                ImuDataMsg, '/krsbi/imu', sensor_qos)
            
            self.motor_fb_pub = self.create_publisher(
                MotorFeedbackMsg, '/krsbi/motor_feedback', sensor_qos)
            
            self.distance_pub = self.create_publisher(
                DistanceSensors, '/krsbi/distance_sensors', sensor_qos)
            
            self.robot_state_pub = self.create_publisher(
                RobotState, '/krsbi/robot_state', 10)
        
        # =================================================================
        # Subscribers
        # =================================================================
        if MSGS_AVAILABLE:
            self.motor_cmd_sub = self.create_subscription(
                MotorCommand, '/krsbi/motor_command',
                self.motor_command_callback, 10)
        
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/krsbi/cmd_vel',
            self.cmd_vel_callback, 10)
        
        self.emergency_sub = self.create_subscription(
            Bool, '/krsbi/emergency_stop',
            self.emergency_callback, 10)
        
        # =================================================================
        # Services
        # =================================================================
        if MSGS_AVAILABLE:
            self.gripper_srv = self.create_service(
                SetGripper, '/krsbi/set_gripper',
                self.handle_set_gripper)
            
            self.charge_srv = self.create_service(
                ChargeKicker, '/krsbi/charge_kicker',
                self.handle_charge_kicker)
            
            self.kick_srv = self.create_service(
                Kick, '/krsbi/kick',
                self.handle_kick)
        
        # =================================================================
        # Timers
        # =================================================================
        # Heartbeat timer
        self.heartbeat_timer = self.create_timer(
            self.heartbeat_interval, self.send_heartbeat)
        
        # Sensor request timer
        sensor_period = 1.0 / self.sensor_rate
        self.sensor_timer = self.create_timer(
            sensor_period, self.request_sensors)
        
        # Connection check timer
        self.connection_timer = self.create_timer(
            1.0, self.check_connection)
        
        # =================================================================
        # Serial Threads
        # =================================================================
        self.read_thread = threading.Thread(target=self.read_loop, daemon=True)
        self.write_thread = threading.Thread(target=self.write_loop, daemon=True)
        
        # =================================================================
        # Connect
        # =================================================================
        self.connect()
        
        # Start threads after connection attempt
        self.read_thread.start()
        self.write_thread.start()
        
        self.get_logger().info(f'Serial Comm Node started (port: {self.port})')
    
    # =====================================================================
    # Connection Management
    # =====================================================================
    
    def connect(self) -> bool:
        """
        Connect to serial port.
        
        Returns:
            True if connected successfully
        """
        if self.serial and self.serial.is_open:
            return True
        
        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout,
                write_timeout=self.timeout,
            )
            
            # Wait for Arduino reset
            time.sleep(2.0)
            
            # Clear buffers
            self.serial.reset_input_buffer()
            self.serial.reset_output_buffer()
            self.parser.clear()
            
            self.is_connected = True
            self.last_heartbeat_time = time.time()
            
            self.get_logger().info(f'Connected to {self.port}')
            self.publish_connection_status(True)
            
            return True
            
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to connect: {e}')
            self.is_connected = False
            self.publish_connection_status(False)
            return False
    
    def disconnect(self):
        """Disconnect from serial port."""
        if self.serial and self.serial.is_open:
            try:
                self.serial.close()
            except Exception:
                pass
        
        self.is_connected = False
        self.publish_connection_status(False)
        self.get_logger().info('Disconnected')
    
    def reconnect(self):
        """Attempt to reconnect."""
        self.disconnect()
        time.sleep(self.reconnect_interval)
        self.connect()
    
    def find_arduino_port(self) -> Optional[str]:
        """
        Auto-detect Arduino Mega port.
        
        Returns:
            Port name if found, None otherwise
        """
        ports = serial.tools.list_ports.comports()
        
        for port in ports:
            # Arduino Mega VID:PID
            if port.vid == 0x2341 and port.pid == 0x0042:
                return port.device
            # Generic FTDI/CH340
            if 'Arduino' in (port.description or ''):
                return port.device
            if 'USB Serial' in (port.description or ''):
                return port.device
        
        return None
    
    def check_connection(self):
        """Periodic connection health check."""
        if not self.is_connected:
            if self.auto_reconnect:
                self.get_logger().info('Attempting reconnection...')
                # Try auto-detect first
                auto_port = self.find_arduino_port()
                if auto_port:
                    self.port = auto_port
                self.connect()
            return
        
        # Check heartbeat timeout
        elapsed = time.time() - self.last_heartbeat_time
        if elapsed > self.heartbeat_timeout:
            self.get_logger().warn('Heartbeat timeout - connection lost')
            self.is_connected = False
            self.publish_connection_status(False)
    
    def publish_connection_status(self, connected: bool):
        """Publish connection status."""
        msg = Bool()
        msg.data = connected
        self.connection_pub.publish(msg)
        
        status = String()
        status.data = "CONNECTED" if connected else "DISCONNECTED"
        self.status_pub.publish(status)
    
    # =====================================================================
    # Read/Write Loops
    # =====================================================================
    
    def read_loop(self):
        """Background thread for reading serial data."""
        while self.is_running:
            if not self.is_connected or not self.serial:
                time.sleep(0.1)
                continue
            
            try:
                # Read available data
                if self.serial.in_waiting > 0:
                    data = self.serial.read(self.serial.in_waiting)
                    
                    if self.debug:
                        self.get_logger().debug(f'RX: {data.hex()}')
                    
                    # Parse packets
                    packets = self.parser.feed(data)
                    
                    for packet in packets:
                        self.handle_packet(packet)
                else:
                    time.sleep(0.001)
                    
            except serial.SerialException as e:
                self.get_logger().error(f'Read error: {e}')
                self.is_connected = False
            except Exception as e:
                self.get_logger().error(f'Read loop error: {e}')
    
    def write_loop(self):
        """Background thread for writing serial data."""
        while self.is_running:
            if not self.is_connected or not self.serial:
                time.sleep(0.1)
                continue
            
            try:
                # Get packet from queue (with timeout)
                packet = self.write_queue.get(timeout=0.1)
                
                if packet is None:
                    continue
                
                # Encode and send
                data = packet.encode()
                
                with self.lock:
                    self.serial.write(data)
                    self.packets_sent += 1
                
                if self.debug:
                    self.get_logger().debug(f'TX: {data.hex()}')
                    
            except queue.Empty:
                continue
            except serial.SerialException as e:
                self.get_logger().error(f'Write error: {e}')
                self.is_connected = False
                self.packets_failed += 1
            except Exception as e:
                self.get_logger().error(f'Write loop error: {e}')
    
    def send_packet(self, packet: Packet, priority: bool = False):
        """
        Queue a packet for sending.
        
        Args:
            packet: Packet to send
            priority: If True, send immediately
        """
        if not self.is_connected:
            return
        
        try:
            if priority:
                # Clear queue and send immediately
                while not self.write_queue.empty():
                    try:
                        self.write_queue.get_nowait()
                    except queue.Empty:
                        break
            
            self.write_queue.put_nowait(packet)
            
        except queue.Full:
            self.get_logger().warn('Write queue full, dropping packet')
    
    # =====================================================================
    # Packet Handling
    # =====================================================================
    
    def handle_packet(self, packet: Packet):
        """
        Process received packet.
        
        Args:
            packet: Received packet
        """
        self.packets_received += 1
        
        if not packet.is_valid:
            self.crc_errors += 1
            self.get_logger().warn('Received packet with invalid CRC')
            return
        
        cmd = packet.command
        
        # Heartbeat response
        if cmd == Response.HEARTBEAT_ACK:
            self.last_heartbeat_time = time.time()
            
        # All sensor data
        elif cmd == Response.SENSOR_ALL:
            sensor_data = parse_sensor_all(packet.data)
            if sensor_data:
                self.last_sensor_data = sensor_data
                self.publish_sensor_data(sensor_data)
                
        # IMU data
        elif cmd == Response.SENSOR_IMU:
            imu_data = parse_imu(packet.data)
            if imu_data:
                self.publish_imu_data(imu_data)
                
        # Motor feedback
        elif cmd == Response.SENSOR_MOTORS:
            motor_data = parse_motor_feedback(packet.data)
            if motor_data:
                self.publish_motor_feedback(motor_data)
                
        # Distance sensors
        elif cmd == Response.SENSOR_DISTANCE:
            dist_data = parse_distance(packet.data)
            if dist_data:
                self.publish_distance_data(dist_data)
                
        # ACK/NACK
        elif cmd == Response.ACK:
            pass  # Command acknowledged
        elif cmd == Response.NACK:
            self.get_logger().warn('Command NACK received')
            
        # Status
        elif cmd == Response.STATUS_ERROR:
            self.get_logger().error(f'Arduino error: {packet.data.hex()}')
        elif cmd == Response.STATUS_WARNING:
            self.get_logger().warn(f'Arduino warning: {packet.data.hex()}')
    
    # =====================================================================
    # Publishers
    # =====================================================================
    
    def publish_sensor_data(self, data: SensorAllData):
        """Publish parsed sensor data."""
        if not MSGS_AVAILABLE:
            return
        
        # Publish individual messages
        self.publish_imu_from_all(data)
        self.publish_motors_from_all(data)
        self.publish_distance_from_all(data)
        
        # Update last sensor time
        self.last_sensor_time = time.time()
    
    def publish_imu_from_all(self, data: SensorAllData):
        """Publish IMU data from all-sensor data."""
        if not MSGS_AVAILABLE:
            return
        
        msg = ImuDataMsg()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'
        
        # Euler angles
        msg.roll = data.roll
        msg.pitch = data.pitch
        msg.yaw = data.yaw
        
        # Angular velocity
        msg.angular_velocity_x = data.gyro_x
        msg.angular_velocity_y = data.gyro_y
        msg.angular_velocity_z = data.gyro_z
        
        # Linear acceleration
        msg.linear_acceleration_x = data.accel_x
        msg.linear_acceleration_y = data.accel_y
        msg.linear_acceleration_z = data.accel_z
        
        self.imu_pub.publish(msg)
    
    def publish_motors_from_all(self, data: SensorAllData):
        """Publish motor feedback from all-sensor data."""
        if not MSGS_AVAILABLE:
            return
        
        msg = MotorFeedbackMsg()
        msg.header.stamp = self.get_clock().now().to_msg()
        
        msg.motor1_ticks = data.motor1_ticks
        msg.motor2_ticks = data.motor2_ticks
        msg.motor3_ticks = data.motor3_ticks
        msg.motor1_rpm = float(data.motor1_rpm)
        msg.motor2_rpm = float(data.motor2_rpm)
        msg.motor3_rpm = float(data.motor3_rpm)
        
        self.motor_fb_pub.publish(msg)
    
    def publish_distance_from_all(self, data: SensorAllData):
        """Publish distance sensors from all-sensor data."""
        if not MSGS_AVAILABLE:
            return
        
        msg = DistanceSensors()
        msg.header.stamp = self.get_clock().now().to_msg()
        
        msg.front = data.distance_front
        msg.front_left = data.distance_front_left
        msg.front_right = data.distance_front_right
        msg.left = data.distance_left
        msg.right = data.distance_right
        msg.rear = data.distance_rear
        
        # Calculate min distance and obstacle detection
        distances = [
            data.distance_front, data.distance_front_left,
            data.distance_front_right, data.distance_left,
            data.distance_right, data.distance_rear
        ]
        msg.min_distance = min(d for d in distances if d > 0.05)
        msg.obstacle_detected = msg.min_distance < 0.30
        
        self.distance_pub.publish(msg)
    
    def publish_imu_data(self, data: ImuData):
        """Publish IMU data."""
        if not MSGS_AVAILABLE:
            return
        
        msg = ImuDataMsg()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'
        
        msg.roll = data.roll
        msg.pitch = data.pitch
        msg.yaw = data.yaw
        msg.angular_velocity_x = data.gyro_x
        msg.angular_velocity_y = data.gyro_y
        msg.angular_velocity_z = data.gyro_z
        msg.linear_acceleration_x = data.accel_x
        msg.linear_acceleration_y = data.accel_y
        msg.linear_acceleration_z = data.accel_z
        
        self.imu_pub.publish(msg)
    
    def publish_motor_feedback(self, data: MotorFeedbackData):
        """Publish motor feedback."""
        if not MSGS_AVAILABLE:
            return
        
        msg = MotorFeedbackMsg()
        msg.header.stamp = self.get_clock().now().to_msg()
        
        msg.motor1_ticks = data.motor1_ticks
        msg.motor2_ticks = data.motor2_ticks
        msg.motor3_ticks = data.motor3_ticks
        msg.motor1_rpm = float(data.motor1_rpm)
        msg.motor2_rpm = float(data.motor2_rpm)
        msg.motor3_rpm = float(data.motor3_rpm)
        
        self.motor_fb_pub.publish(msg)
    
    def publish_distance_data(self, data: DistanceData):
        """Publish distance sensors."""
        if not MSGS_AVAILABLE:
            return
        
        msg = DistanceSensors()
        msg.header.stamp = self.get_clock().now().to_msg()
        
        msg.front = data.front
        msg.front_left = data.front_left
        msg.front_right = data.front_right
        msg.left = data.left
        msg.right = data.right
        msg.rear = data.rear
        
        self.distance_pub.publish(msg)
    
    # =====================================================================
    # Callbacks
    # =====================================================================
    
    def motor_command_callback(self, msg: 'MotorCommand'):
        """Handle motor command message."""
        if self.emergency_stop_active:
            return
        
        if msg.emergency_stop:
            self.send_emergency_stop()
            return
        
        if not msg.enable:
            self.send_packet(build_motor_stop())
            return
        
        if msg.mode == 0:  # Velocity mode
            packet = build_motor_velocity(
                msg.linear_x, msg.linear_y, msg.angular_z)
        else:  # RPM mode
            packet = build_motor_rpm(
                int(msg.motor1_rpm),
                int(msg.motor2_rpm),
                int(msg.motor3_rpm))
        
        self.send_packet(packet)
    
    def cmd_vel_callback(self, msg: Twist):
        """Handle cmd_vel message."""
        if self.emergency_stop_active:
            return
        
        packet = build_motor_velocity(
            msg.linear.x,
            msg.linear.y,
            msg.angular.z
        )
        self.send_packet(packet)
    
    def emergency_callback(self, msg: Bool):
        """Handle emergency stop message."""
        if msg.data:
            self.send_emergency_stop()
        else:
            self.emergency_stop_active = False
    
    # =====================================================================
    # Service Handlers
    # =====================================================================
    
    def handle_set_gripper(self, request, response):
        """Handle set gripper service."""
        action = request.action
        
        if action == 0:  # OPEN
            packet = build_gripper_open()
        elif action == 1:  # CLOSE
            packet = build_gripper_close()
        elif action == 5:  # SET_POSITION
            packet = build_gripper_position(int(request.target_position * 255))
        else:
            response.success = False
            response.message = "Unknown action"
            return response
        
        self.send_packet(packet)
        response.success = True
        response.message = "Command sent"
        return response
    
    def handle_charge_kicker(self, request, response):
        """Handle charge kicker service."""
        power = int(request.target_voltage / 200.0 * 100)  # Convert to percentage
        packet = build_kicker_charge(power)
        self.send_packet(packet)
        
        response.success = True
        response.message = f"Charging to {power}%"
        response.estimated_time = 5.0  # Rough estimate
        return response
    
    def handle_kick(self, request, response):
        """Handle kick service."""
        power = int(request.power_level * 100)
        duration = int(request.discharge_duration * 1000)
        
        packet = build_kicker_kick(power, duration)
        self.send_packet(packet, priority=True)
        
        response.success = True
        response.message = f"Kick executed at {power}%"
        return response
    
    # =====================================================================
    # Periodic Tasks
    # =====================================================================
    
    def send_heartbeat(self):
        """Send heartbeat packet."""
        if self.is_connected:
            self.send_packet(build_heartbeat())
    
    def request_sensors(self):
        """Request sensor data from Arduino."""
        if self.is_connected:
            self.send_packet(build_request_sensors())
    
    def send_emergency_stop(self):
        """Send emergency stop command."""
        self.emergency_stop_active = True
        self.send_packet(build_emergency_stop(), priority=True)
        self.get_logger().warn('EMERGENCY STOP activated')
    
    # =====================================================================
    # Lifecycle
    # =====================================================================
    
    def destroy_node(self):
        """Clean up on shutdown."""
        self.is_running = False
        self.disconnect()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    node = SerialCommNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
