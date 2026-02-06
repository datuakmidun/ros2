"""
KRSBI-B Serial Communication Package

Serial communication layer between Intel NUC (ROS 2) and Arduino Mega.

This package provides:
- Serial node for bidirectional communication
- Custom protocol for commands and sensor data
- CRC validation for data integrity
- Automatic reconnection and error handling
"""

from .protocol import (
    Command,
    Response,
    Packet,
    PacketParser,
)

from .crc_utils import (
    crc8,
    validate_crc,
)

__all__ = [
    # Protocol
    'Command',
    'Response',
    'Packet',
    'PacketParser',
    # CRC
    'crc8',
    'validate_crc',
]

__version__ = '1.0.0'
__author__ = 'KRSBI-B Team'
