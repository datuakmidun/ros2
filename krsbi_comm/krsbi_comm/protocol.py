"""
KRSBI-B Soccer Robot - Communication Protocol

Packet format and command definitions for Arduino communication.

Packet Structure:
+--------+--------+--------+--------+...+--------+--------+
| START  |  LEN   |  CMD   | DATA_0 |...| DATA_N |  CRC   |
+--------+--------+--------+--------+...+--------+--------+
|  0xAA  | 1 byte | 1 byte |    N bytes    | 1 byte |
+--------+--------+--------+-----------------+--------+
"""

from dataclasses import dataclass, field
from enum import IntEnum
from typing import Optional, List, Tuple, Union
import struct

from .crc_utils import crc8, validate_crc


# =============================================================================
# Constants
# =============================================================================

START_BYTE = 0xAA
MIN_PACKET_LENGTH = 4  # START + LEN + CMD + CRC
MAX_PACKET_LENGTH = 64
HEADER_SIZE = 3  # START + LEN + CMD


# =============================================================================
# Command Definitions (PC -> Arduino)
# =============================================================================

class Command(IntEnum):
    """Commands sent from PC to Arduino."""
    
    # Motor control (0x01 - 0x0F)
    MOTOR_VELOCITY = 0x01      # Set motor velocities (vx, vy, omega)
    MOTOR_RPM = 0x02           # Set motor RPM directly
    MOTOR_PWM = 0x03           # Set motor PWM directly
    MOTOR_STOP = 0x04          # Stop all motors
    
    # Kicker control (0x10 - 0x1F)
    KICKER_CHARGE = 0x10       # Start charging capacitor
    KICKER_KICK = 0x11         # Execute kick
    KICKER_STOP = 0x12         # Stop charging/discharge
    
    # Gripper control (0x20 - 0x2F)
    GRIPPER_OPEN = 0x20        # Open gripper
    GRIPPER_CLOSE = 0x21       # Close gripper
    GRIPPER_SET_POS = 0x22     # Set gripper position
    GRIPPER_STOP = 0x23        # Stop gripper motor
    
    # Sensor requests (0x30 - 0x3F)
    REQUEST_SENSORS = 0x30     # Request all sensor data
    REQUEST_IMU = 0x31         # Request IMU data only
    REQUEST_MOTORS = 0x32      # Request motor feedback
    REQUEST_DISTANCE = 0x33    # Request distance sensors
    REQUEST_STATUS = 0x34      # Request system status
    
    # Calibration (0x40 - 0x4F)
    CALIBRATE_IMU = 0x40       # Calibrate IMU
    CALIBRATE_MOTORS = 0x41    # Reset encoder counts
    
    # System (0xF0 - 0xFF)
    HEARTBEAT = 0xF0           # Heartbeat ping
    EMERGENCY_STOP = 0xFF      # Emergency stop all


# =============================================================================
# Response Definitions (Arduino -> PC)
# =============================================================================

class Response(IntEnum):
    """Responses sent from Arduino to PC."""
    
    # Acknowledgments (0x00 - 0x0F)
    ACK = 0x00                 # Command acknowledged
    NACK = 0x01                # Command failed
    
    # Sensor data (0x80 - 0x8F)
    SENSOR_ALL = 0x80          # All sensor data
    SENSOR_IMU = 0x81          # IMU data
    SENSOR_MOTORS = 0x82       # Motor feedback
    SENSOR_DISTANCE = 0x83     # Distance sensors
    SENSOR_KICKER = 0x84       # Kicker state
    SENSOR_GRIPPER = 0x85      # Gripper state
    SENSOR_BATTERY = 0x86      # Battery status
    
    # Status (0x90 - 0x9F)
    STATUS_OK = 0x90           # System OK
    STATUS_ERROR = 0x91        # Error occurred
    STATUS_WARNING = 0x92      # Warning
    
    # System (0xF0 - 0xFF)
    HEARTBEAT_ACK = 0xF0       # Heartbeat response


# =============================================================================
# Error Codes
# =============================================================================

class ErrorCode(IntEnum):
    """Error codes returned by Arduino."""
    
    NONE = 0x00
    INVALID_COMMAND = 0x01
    INVALID_LENGTH = 0x02
    CRC_ERROR = 0x03
    TIMEOUT = 0x04
    MOTOR_STALL = 0x10
    MOTOR_OVERCURRENT = 0x11
    KICKER_OVERVOLTAGE = 0x20
    KICKER_TIMEOUT = 0x21
    IMU_ERROR = 0x30
    SERIAL_ERROR = 0x40
    EMERGENCY_ACTIVE = 0xFF


# =============================================================================
# Packet Class
# =============================================================================

@dataclass
class Packet:
    """
    Communication packet.
    
    Attributes:
        command: Command or response byte
        data: Payload data
        is_valid: Whether packet CRC is valid
    """
    
    command: int
    data: bytes = field(default_factory=bytes)
    is_valid: bool = True
    
    @property
    def length(self) -> int:
        """Total packet length including header and CRC."""
        return HEADER_SIZE + len(self.data) + 1
    
    def encode(self) -> bytes:
        """
        Encode packet to bytes for transmission.
        
        Returns:
            Complete packet with header and CRC
        """
        # Build packet without CRC
        packet = bytearray([
            START_BYTE,
            self.length,
            self.command,
        ])
        packet.extend(self.data)
        
        # Calculate and append CRC (over LEN + CMD + DATA)
        crc = crc8(packet[1:])  # Exclude START_BYTE from CRC
        packet.append(crc)
        
        return bytes(packet)
    
    @classmethod
    def decode(cls, data: bytes) -> Optional['Packet']:
        """
        Decode packet from received bytes.
        
        Args:
            data: Raw packet bytes (must include START, LEN, CMD, DATA, CRC)
            
        Returns:
            Decoded Packet or None if invalid
        """
        if len(data) < MIN_PACKET_LENGTH:
            return None
        
        if data[0] != START_BYTE:
            return None
        
        length = data[1]
        if len(data) < length:
            return None
        
        command = data[2]
        payload = data[3:length - 1]
        received_crc = data[length - 1]
        
        # Validate CRC (over LEN + CMD + DATA)
        calculated_crc = crc8(data[1:length - 1])
        is_valid = calculated_crc == received_crc
        
        return cls(
            command=command,
            data=bytes(payload),
            is_valid=is_valid,
        )
    
    def __repr__(self) -> str:
        cmd_name = self._get_command_name()
        return f"Packet(cmd={cmd_name}, data={self.data.hex()}, valid={self.is_valid})"
    
    def _get_command_name(self) -> str:
        """Get human-readable command name."""
        try:
            if self.command < 0x80:
                return Command(self.command).name
            else:
                return Response(self.command).name
        except ValueError:
            return f"0x{self.command:02X}"


# =============================================================================
# Packet Builder Functions
# =============================================================================

def build_motor_velocity(vx: float, vy: float, omega: float) -> Packet:
    """
    Build motor velocity command packet.
    
    Args:
        vx: Linear velocity X in m/s
        vy: Linear velocity Y in m/s
        omega: Angular velocity in rad/s
        
    Returns:
        Encoded packet
    """
    # Convert to mm/s and mrad/s (int16 range: -32768 to 32767)
    vx_mm = int(vx * 1000)
    vy_mm = int(vy * 1000)
    omega_mrad = int(omega * 1000)
    
    # Clamp values
    vx_mm = max(-32767, min(32767, vx_mm))
    vy_mm = max(-32767, min(32767, vy_mm))
    omega_mrad = max(-32767, min(32767, omega_mrad))
    
    data = struct.pack('>hhh', vx_mm, vy_mm, omega_mrad)
    return Packet(command=Command.MOTOR_VELOCITY, data=data)


def build_motor_rpm(m1: int, m2: int, m3: int) -> Packet:
    """
    Build motor RPM command packet.
    
    Args:
        m1, m2, m3: Motor RPM values (-200 to 200)
        
    Returns:
        Encoded packet
    """
    data = struct.pack('>hhh', m1, m2, m3)
    return Packet(command=Command.MOTOR_RPM, data=data)


def build_motor_pwm(m1: int, m2: int, m3: int) -> Packet:
    """
    Build motor PWM command packet.
    
    Args:
        m1, m2, m3: Motor PWM values (-255 to 255)
        
    Returns:
        Encoded packet
    """
    data = struct.pack('>hhh', m1, m2, m3)
    return Packet(command=Command.MOTOR_PWM, data=data)


def build_motor_stop() -> Packet:
    """Build motor stop command packet."""
    return Packet(command=Command.MOTOR_STOP)


def build_kicker_charge(power_percent: int) -> Packet:
    """
    Build kicker charge command packet.
    
    Args:
        power_percent: Target charge level (0-100%)
        
    Returns:
        Encoded packet
    """
    power = max(0, min(100, power_percent))
    data = struct.pack('>B', power)
    return Packet(command=Command.KICKER_CHARGE, data=data)


def build_kicker_kick(power: int, duration_ms: int = 50) -> Packet:
    """
    Build kicker kick command packet.
    
    Args:
        power: Kick power (0-100%)
        duration_ms: Kick duration in milliseconds
        
    Returns:
        Encoded packet
    """
    power = max(0, min(100, power))
    duration = max(10, min(255, duration_ms))
    data = struct.pack('>BB', power, duration)
    return Packet(command=Command.KICKER_KICK, data=data)


def build_kicker_stop() -> Packet:
    """Build kicker stop command packet."""
    return Packet(command=Command.KICKER_STOP)


def build_gripper_open() -> Packet:
    """Build gripper open command packet."""
    return Packet(command=Command.GRIPPER_OPEN)


def build_gripper_close() -> Packet:
    """Build gripper close command packet."""
    return Packet(command=Command.GRIPPER_CLOSE)


def build_gripper_position(position: int) -> Packet:
    """
    Build gripper position command packet.
    
    Args:
        position: Gripper position (0=closed, 255=open)
        
    Returns:
        Encoded packet
    """
    pos = max(0, min(255, position))
    data = struct.pack('>B', pos)
    return Packet(command=Command.GRIPPER_SET_POS, data=data)


def build_request_sensors() -> Packet:
    """Build request all sensors command packet."""
    return Packet(command=Command.REQUEST_SENSORS)


def build_request_imu() -> Packet:
    """Build request IMU command packet."""
    return Packet(command=Command.REQUEST_IMU)


def build_request_motors() -> Packet:
    """Build request motor feedback command packet."""
    return Packet(command=Command.REQUEST_MOTORS)


def build_request_distance() -> Packet:
    """Build request distance sensors command packet."""
    return Packet(command=Command.REQUEST_DISTANCE)


def build_heartbeat() -> Packet:
    """Build heartbeat command packet."""
    return Packet(command=Command.HEARTBEAT)


def build_emergency_stop() -> Packet:
    """Build emergency stop command packet."""
    return Packet(command=Command.EMERGENCY_STOP)


# =============================================================================
# Packet Parser (for receiving)
# =============================================================================

class PacketParser:
    """
    Incremental packet parser for serial data streams.
    
    Handles partial packets and finds valid packets in byte stream.
    """
    
    def __init__(self, buffer_size: int = 512):
        """
        Initialize parser.
        
        Args:
            buffer_size: Maximum buffer size
        """
        self._buffer = bytearray()
        self._buffer_size = buffer_size
    
    def feed(self, data: bytes) -> List[Packet]:
        """
        Feed data to parser and extract complete packets.
        
        Args:
            data: Incoming bytes
            
        Returns:
            List of complete, valid packets
        """
        self._buffer.extend(data)
        
        # Limit buffer size
        if len(self._buffer) > self._buffer_size:
            # Find last START_BYTE and trim
            for i in range(len(self._buffer) - 1, -1, -1):
                if self._buffer[i] == START_BYTE:
                    self._buffer = self._buffer[i:]
                    break
            else:
                self._buffer.clear()
        
        packets = []
        
        while True:
            packet = self._try_parse()
            if packet is None:
                break
            packets.append(packet)
        
        return packets
    
    def _try_parse(self) -> Optional[Packet]:
        """
        Try to parse a complete packet from buffer.
        
        Returns:
            Parsed packet or None
        """
        # Find START_BYTE
        while len(self._buffer) > 0 and self._buffer[0] != START_BYTE:
            self._buffer.pop(0)
        
        # Need at least MIN_PACKET_LENGTH bytes
        if len(self._buffer) < MIN_PACKET_LENGTH:
            return None
        
        # Get length
        length = self._buffer[1]
        
        # Validate length
        if length < MIN_PACKET_LENGTH or length > MAX_PACKET_LENGTH:
            # Invalid length, skip this START_BYTE
            self._buffer.pop(0)
            return None
        
        # Wait for complete packet
        if len(self._buffer) < length:
            return None
        
        # Extract packet data
        packet_data = bytes(self._buffer[:length])
        
        # Parse packet
        packet = Packet.decode(packet_data)
        
        if packet and packet.is_valid:
            # Remove parsed bytes from buffer
            self._buffer = self._buffer[length:]
            return packet
        else:
            # Invalid packet, skip START_BYTE and try again
            self._buffer.pop(0)
            return None
    
    def clear(self):
        """Clear the parser buffer."""
        self._buffer.clear()
    
    @property
    def buffer_length(self) -> int:
        """Current buffer length."""
        return len(self._buffer)


# =============================================================================
# Response Parsers
# =============================================================================

@dataclass
class SensorAllData:
    """Parsed all-sensor response data."""
    # IMU
    roll: float        # radians
    pitch: float       # radians
    yaw: float         # radians
    gyro_x: float      # rad/s
    gyro_y: float      # rad/s
    gyro_z: float      # rad/s
    accel_x: float     # m/s²
    accel_y: float     # m/s²
    accel_z: float     # m/s²
    
    # Motors
    motor1_ticks: int
    motor2_ticks: int
    motor3_ticks: int
    motor1_rpm: int
    motor2_rpm: int
    motor3_rpm: int
    
    # Distance sensors (meters)
    distance_front: float
    distance_front_left: float
    distance_front_right: float
    distance_left: float
    distance_right: float
    distance_rear: float
    
    # Status
    battery_voltage: float
    flags: int


def parse_sensor_all(data: bytes) -> Optional[SensorAllData]:
    """
    Parse all-sensor response data.
    
    Expected format: 48 bytes
    - IMU: 18 bytes (9x int16)
    - Motors: 18 bytes (3x int32 + 3x int16)
    - Distance: 12 bytes (6x uint16)
    
    Args:
        data: Payload data
        
    Returns:
        Parsed sensor data or None
    """
    if len(data) < 48:
        return None
    
    try:
        # Parse IMU (9x int16, scaled)
        imu = struct.unpack('>hhhhhhhhh', data[0:18])
        roll = imu[0] / 100.0 * 0.0174533      # 0.01 deg to rad
        pitch = imu[1] / 100.0 * 0.0174533
        yaw = imu[2] / 100.0 * 0.0174533
        gyro_x = imu[3] / 100.0 * 0.0174533    # 0.01 deg/s to rad/s
        gyro_y = imu[4] / 100.0 * 0.0174533
        gyro_z = imu[5] / 100.0 * 0.0174533
        accel_x = imu[6] / 1000.0 * 9.81       # mg to m/s²
        accel_y = imu[7] / 1000.0 * 9.81
        accel_z = imu[8] / 1000.0 * 9.81
        
        # Parse motors (3x int32 ticks + 3x int16 rpm)
        motors = struct.unpack('>ihihih', data[18:36])
        m1_ticks, m1_rpm = motors[0], motors[1]
        m2_ticks, m2_rpm = motors[2], motors[3]
        m3_ticks, m3_rpm = motors[4], motors[5]
        
        # Parse distance (6x uint16 in mm)
        dist = struct.unpack('>HHHHHH', data[36:48])
        dist_m = [d / 1000.0 for d in dist]
        
        # Parse status if available
        battery_voltage = 0.0
        flags = 0
        if len(data) >= 52:
            status = struct.unpack('>HH', data[48:52])
            battery_voltage = status[0] / 100.0  # 0.01V units
            flags = status[1]
        
        return SensorAllData(
            roll=roll, pitch=pitch, yaw=yaw,
            gyro_x=gyro_x, gyro_y=gyro_y, gyro_z=gyro_z,
            accel_x=accel_x, accel_y=accel_y, accel_z=accel_z,
            motor1_ticks=m1_ticks, motor2_ticks=m2_ticks, motor3_ticks=m3_ticks,
            motor1_rpm=m1_rpm, motor2_rpm=m2_rpm, motor3_rpm=m3_rpm,
            distance_front=dist_m[0], distance_front_left=dist_m[1],
            distance_front_right=dist_m[2], distance_left=dist_m[3],
            distance_right=dist_m[4], distance_rear=dist_m[5],
            battery_voltage=battery_voltage,
            flags=flags,
        )
    except struct.error:
        return None


@dataclass
class ImuData:
    """Parsed IMU response data."""
    roll: float
    pitch: float
    yaw: float
    gyro_x: float
    gyro_y: float
    gyro_z: float
    accel_x: float
    accel_y: float
    accel_z: float


def parse_imu(data: bytes) -> Optional[ImuData]:
    """Parse IMU response data (18 bytes)."""
    if len(data) < 18:
        return None
    
    try:
        values = struct.unpack('>hhhhhhhhh', data[0:18])
        return ImuData(
            roll=values[0] / 100.0 * 0.0174533,
            pitch=values[1] / 100.0 * 0.0174533,
            yaw=values[2] / 100.0 * 0.0174533,
            gyro_x=values[3] / 100.0 * 0.0174533,
            gyro_y=values[4] / 100.0 * 0.0174533,
            gyro_z=values[5] / 100.0 * 0.0174533,
            accel_x=values[6] / 1000.0 * 9.81,
            accel_y=values[7] / 1000.0 * 9.81,
            accel_z=values[8] / 1000.0 * 9.81,
        )
    except struct.error:
        return None


@dataclass
class MotorFeedbackData:
    """Parsed motor feedback response data."""
    motor1_ticks: int
    motor1_rpm: int
    motor2_ticks: int
    motor2_rpm: int
    motor3_ticks: int
    motor3_rpm: int


def parse_motor_feedback(data: bytes) -> Optional[MotorFeedbackData]:
    """Parse motor feedback response data (18 bytes)."""
    if len(data) < 18:
        return None
    
    try:
        values = struct.unpack('>ihihih', data[0:18])
        return MotorFeedbackData(
            motor1_ticks=values[0], motor1_rpm=values[1],
            motor2_ticks=values[2], motor2_rpm=values[3],
            motor3_ticks=values[4], motor3_rpm=values[5],
        )
    except struct.error:
        return None


@dataclass
class DistanceData:
    """Parsed distance sensors response data."""
    front: float
    front_left: float
    front_right: float
    left: float
    right: float
    rear: float


def parse_distance(data: bytes) -> Optional[DistanceData]:
    """Parse distance sensors response data (12 bytes)."""
    if len(data) < 12:
        return None
    
    try:
        values = struct.unpack('>HHHHHH', data[0:12])
        return DistanceData(
            front=values[0] / 1000.0,
            front_left=values[1] / 1000.0,
            front_right=values[2] / 1000.0,
            left=values[3] / 1000.0,
            right=values[4] / 1000.0,
            rear=values[5] / 1000.0,
        )
    except struct.error:
        return None
