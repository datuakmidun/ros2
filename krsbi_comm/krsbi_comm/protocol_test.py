#!/usr/bin/env python3
"""
KRSBI-B Soccer Robot - Protocol Test

Test script for protocol encoding/decoding without hardware.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import struct
import time

from .protocol import (
    Command, Response, Packet, PacketParser,
    build_motor_velocity, build_motor_rpm, build_motor_pwm, build_motor_stop,
    build_kicker_charge, build_kicker_kick,
    build_gripper_open, build_gripper_close, build_gripper_position,
    build_request_sensors, build_heartbeat, build_emergency_stop,
    parse_sensor_all, parse_imu, parse_motor_feedback, parse_distance,
    START_BYTE,
)
from .crc_utils import crc8


class ProtocolTestNode(Node):
    """
    Protocol test node for debugging without hardware.
    """
    
    def __init__(self):
        super().__init__('protocol_test')
        
        self.status_pub = self.create_publisher(String, '/krsbi/test/status', 10)
        
        self.get_logger().info('Protocol Test Node started')
        
        # Run tests
        self.run_all_tests()
    
    def log_result(self, test_name: str, passed: bool, details: str = ''):
        """Log test result."""
        status = "PASS" if passed else "FAIL"
        msg = f"[{status}] {test_name}"
        if details:
            msg += f": {details}"
        
        if passed:
            self.get_logger().info(msg)
        else:
            self.get_logger().error(msg)
        
        # Publish status
        status_msg = String()
        status_msg.data = msg
        self.status_pub.publish(status_msg)
    
    def run_all_tests(self):
        """Run all protocol tests."""
        self.get_logger().info('=' * 60)
        self.get_logger().info('Running Protocol Tests')
        self.get_logger().info('=' * 60)
        
        tests = [
            self.test_crc,
            self.test_packet_encode,
            self.test_packet_decode,
            self.test_motor_commands,
            self.test_kicker_commands,
            self.test_gripper_commands,
            self.test_sensor_parsing,
            self.test_packet_parser,
        ]
        
        passed = 0
        failed = 0
        
        for test in tests:
            try:
                result = test()
                if result:
                    passed += 1
                else:
                    failed += 1
            except Exception as e:
                self.log_result(test.__name__, False, str(e))
                failed += 1
        
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'Tests Complete: {passed} passed, {failed} failed')
        self.get_logger().info('=' * 60)
    
    def test_crc(self) -> bool:
        """Test CRC calculation."""
        # Test known data
        data = bytes([0x05, 0x01, 0x00, 0x00])
        crc_val = crc8(data)
        
        # Verify CRC is consistent
        crc_val2 = crc8(data)
        
        passed = crc_val == crc_val2
        self.log_result('CRC Calculation', passed, f'CRC=0x{crc_val:02X}')
        return passed
    
    def test_packet_encode(self) -> bool:
        """Test packet encoding."""
        # Create simple packet
        packet = Packet(command=Command.HEARTBEAT)
        encoded = packet.encode()
        
        # Verify structure
        # START(1) + LEN(1) + CMD(1) + CRC(1) = 4 bytes
        passed = (
            len(encoded) == 4 and
            encoded[0] == START_BYTE and
            encoded[1] == 4 and  # Length
            encoded[2] == Command.HEARTBEAT
        )
        
        self.log_result('Packet Encode', passed, f'Data: {encoded.hex()}')
        return passed
    
    def test_packet_decode(self) -> bool:
        """Test packet decoding (roundtrip)."""
        # Create and encode packet
        original = Packet(command=Command.MOTOR_VELOCITY, data=b'\x00\x01\x00\x02\x00\x03')
        encoded = original.encode()
        
        # Decode
        decoded = Packet.decode(encoded)
        
        passed = (
            decoded is not None and
            decoded.is_valid and
            decoded.command == original.command and
            decoded.data == original.data
        )
        
        self.log_result('Packet Decode', passed, f'Valid={decoded.is_valid if decoded else False}')
        return passed
    
    def test_motor_commands(self) -> bool:
        """Test motor command builders."""
        # Velocity command
        pkt1 = build_motor_velocity(1.0, 0.5, 0.1)
        data1 = struct.unpack('>hhh', pkt1.data)
        
        # RPM command
        pkt2 = build_motor_rpm(100, 100, 100)
        data2 = struct.unpack('>hhh', pkt2.data)
        
        # Stop command
        pkt3 = build_motor_stop()
        
        passed = (
            pkt1.command == Command.MOTOR_VELOCITY and
            data1 == (1000, 500, 100) and  # Scaled values
            pkt2.command == Command.MOTOR_RPM and
            data2 == (100, 100, 100) and
            pkt3.command == Command.MOTOR_STOP
        )
        
        self.log_result('Motor Commands', passed)
        return passed
    
    def test_kicker_commands(self) -> bool:
        """Test kicker command builders."""
        # Charge command
        pkt1 = build_kicker_charge(80)
        power = struct.unpack('>B', pkt1.data)[0]
        
        # Kick command
        pkt2 = build_kicker_kick(100, 50)
        kick_data = struct.unpack('>BB', pkt2.data)
        
        passed = (
            pkt1.command == Command.KICKER_CHARGE and
            power == 80 and
            pkt2.command == Command.KICKER_KICK and
            kick_data == (100, 50)
        )
        
        self.log_result('Kicker Commands', passed)
        return passed
    
    def test_gripper_commands(self) -> bool:
        """Test gripper command builders."""
        pkt1 = build_gripper_open()
        pkt2 = build_gripper_close()
        pkt3 = build_gripper_position(128)
        
        passed = (
            pkt1.command == Command.GRIPPER_OPEN and
            pkt2.command == Command.GRIPPER_CLOSE and
            pkt3.command == Command.GRIPPER_SET_POS and
            pkt3.data[0] == 128
        )
        
        self.log_result('Gripper Commands', passed)
        return passed
    
    def test_sensor_parsing(self) -> bool:
        """Test sensor data parsing."""
        # Create fake sensor data (48 bytes minimum)
        # IMU: 9x int16 = 18 bytes
        # Motors: 3x (int32 + int16) = 18 bytes
        # Distance: 6x uint16 = 12 bytes
        
        imu_data = struct.pack('>hhhhhhhhh', 
            1000, 500, 0,      # roll, pitch, yaw (0.01 deg)
            100, 100, 100,     # gyro x, y, z
            0, 0, 1000         # accel x, y, z (mg)
        )
        
        motor_data = struct.pack('>ihihih',
            1000, 50,          # motor1 ticks, rpm
            1000, 50,          # motor2
            1000, 50           # motor3
        )
        
        dist_data = struct.pack('>HHHHHH',
            300, 400, 400,     # front, front_left, front_right (mm)
            500, 500, 800      # left, right, rear (mm)
        )
        
        full_data = imu_data + motor_data + dist_data
        
        # Parse
        parsed = parse_sensor_all(full_data)
        
        passed = (
            parsed is not None and
            abs(parsed.roll - 0.1745) < 0.01 and  # 10 deg in rad
            parsed.motor1_ticks == 1000 and
            abs(parsed.distance_front - 0.3) < 0.01
        )
        
        self.log_result('Sensor Parsing', passed, 
            f'Roll={parsed.roll:.3f} Motor1={parsed.motor1_ticks} Dist={parsed.distance_front:.2f}' if parsed else 'Failed')
        return passed
    
    def test_packet_parser(self) -> bool:
        """Test incremental packet parser."""
        parser = PacketParser()
        
        # Create multiple packets
        pkt1 = build_heartbeat()
        pkt2 = build_motor_velocity(0.5, 0.0, 0.0)
        
        # Simulate receiving data in chunks
        data = pkt1.encode() + pkt2.encode()
        
        # Feed in parts
        packets = []
        packets.extend(parser.feed(data[:3]))  # Partial
        packets.extend(parser.feed(data[3:]))   # Rest
        
        passed = len(packets) == 2
        
        self.log_result('Packet Parser', passed, f'Parsed {len(packets)} packets')
        return passed


def main(args=None):
    rclpy.init(args=args)
    
    node = ProtocolTestNode()
    
    # Spin briefly to publish results
    for _ in range(10):
        rclpy.spin_once(node, timeout_sec=0.1)
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
