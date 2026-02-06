# Changelog

All notable changes to the `krsbi_comm` package will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [1.0.0] - 2026-02-06

### Added

#### Protocol Implementation

- **protocol.py** - Complete packet protocol:
  - Packet encoding/decoding with START byte (0xAA)
  - CRC-8 checksum validation
  - Command definitions (motor, kicker, gripper, sensors)
  - Response definitions and parsers
  - PacketParser for incremental stream parsing

- **crc_utils.py** - CRC-8 utilities:
  - Lookup table for fast CRC calculation
  - Validation and verification functions
  - Polynomial 0x07 (standard CRC-8)

#### Serial Node

- **serial_node.py** - Main ROS 2 node:
  - Async read/write threads for low latency
  - Auto-reconnection with port detection
  - Heartbeat mechanism for connection monitoring
  - Publishers for all sensor data (IMU, motors, distance)
  - Subscribers for motor commands and cmd_vel
  - Services for gripper and kicker control
  - Emergency stop support

#### Configuration

- **serial_config.yaml** - Serial port settings:
  - Port configuration with auto-detect
  - Reconnection parameters
  - Rate limits and buffer sizes

- **protocol_config.yaml** - Protocol specification:
  - Complete command/response definitions
  - Data format specifications
  - Error codes

#### Testing

- **protocol_test.py** - Protocol test node:
  - CRC calculation tests
  - Packet encode/decode roundtrip
  - Command builder verification
  - Sensor parser verification

#### Launch

- **comm_bringup.launch.py** - Communication startup

### Commands Implemented

| Category | Commands                                                       |
| -------- | -------------------------------------------------------------- |
| Motor    | VELOCITY, RPM, PWM, STOP                                       |
| Kicker   | CHARGE, KICK, STOP                                             |
| Gripper  | OPEN, CLOSE, SET_POS, STOP                                     |
| Sensor   | REQUEST_SENSORS, REQUEST_IMU, REQUEST_MOTORS, REQUEST_DISTANCE |
| System   | HEARTBEAT, EMERGENCY_STOP                                      |

### Response Parsers

- All sensor data (48+ bytes)
- IMU data (18 bytes)
- Motor feedback (18 bytes)
- Distance sensors (12 bytes)

---

## [Unreleased]

### Planned

- Micro-ROS bridge option
- Service for direct PWM control
- Diagnostic aggregator integration
- Bandwidth monitoring
- Packet logging to file

---

## Version History

| Version | Date       | Description                                |
| ------- | ---------- | ------------------------------------------ |
| 1.0.0   | 2026-02-06 | Initial release - Serial protocol and node |
