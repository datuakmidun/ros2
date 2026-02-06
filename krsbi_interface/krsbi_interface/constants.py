"""
KRSBI-B Soccer Robot - Constants and Enumerations

Game rules, robot constraints, and field specifications
following KRSBI-B (RoboCup Middle Size League) regulations.
"""

from enum import IntEnum, Enum
from dataclasses import dataclass
from typing import Tuple


# =============================================================================
# Game State Enumerations
# =============================================================================

class GameState(IntEnum):
    """Main game states from GameController."""
    INITIAL = 0      # Before game starts
    READY = 1        # Robots move to starting positions
    SET = 2          # Robots stop and wait for kickoff
    PLAYING = 3      # Game is active
    FINISHED = 4     # Game/half has ended


class SetPlay(IntEnum):
    """Secondary game states (set plays)."""
    NORMAL = 0       # Normal play
    KICKOFF = 1      # Kickoff
    FREEKICK = 2     # Free kick
    PENALTY = 3      # Penalty kick
    CORNER = 4       # Corner kick
    THROW_IN = 5     # Throw in
    GOAL_KICK = 6    # Goal kick
    DROPPED_BALL = 7 # Dropped ball


class RobotRole(IntEnum):
    """Robot roles during game."""
    GOALKEEPER = 0
    STRIKER = 1
    DEFENDER = 2
    SUPPORT = 3


class TeamColor(IntEnum):
    """Team colors."""
    BLUE = 0
    YELLOW = 1


class PenaltyType(IntEnum):
    """Penalty types."""
    NONE = 0
    BALL_MANIPULATION = 1
    PHYSICAL_CONTACT = 2
    ILLEGAL_ATTACK = 3
    ILLEGAL_DEFENSE = 4
    PICKUP_BALL = 5
    PUSHING = 6
    LEAVING_FIELD = 7
    MOTION_IN_SET = 8
    INACTIVE = 9
    ILLEGAL_POSITION = 10


# =============================================================================
# Hardware Constants
# =============================================================================

class MotorID(IntEnum):
    """Motor identifiers."""
    FRONT = 0        # M1: Front (0°)
    REAR_LEFT = 1    # M2: Rear-left (120°)
    REAR_RIGHT = 2   # M3: Rear-right (240°)


class CameraSource(IntEnum):
    """Camera identifiers (matching krsbi_msgs)."""
    OMNI = 0         # Fisheye 360°
    FRONT = 1        # Logitech webcam
    FUSED = 2        # Fused from both


class KickerState(IntEnum):
    """Kicker system states (matching krsbi_msgs)."""
    IDLE = 0
    CHARGING = 1
    READY = 2
    KICKING = 3
    COOLDOWN = 4
    ERROR = 5


class GripperAction(IntEnum):
    """Gripper action commands (matching krsbi_msgs)."""
    OPEN = 0
    CLOSE = 1
    GRAB = 2
    RELEASE = 3
    STOP = 4
    SET_POSITION = 5


class PowerLevel(IntEnum):
    """Kicker power levels (matching krsbi_msgs)."""
    LOW = 0
    MEDIUM = 1
    HIGH = 2
    MAX = 3


# =============================================================================
# Physical Constants
# =============================================================================

@dataclass(frozen=True)
class FieldDimensions:
    """Standard KRSBI-B field dimensions in meters."""
    LENGTH: float = 9.0
    WIDTH: float = 6.0
    GOAL_WIDTH: float = 2.6
    GOAL_DEPTH: float = 0.5
    GOAL_AREA_LENGTH: float = 0.75
    GOAL_AREA_WIDTH: float = 3.5
    PENALTY_AREA_LENGTH: float = 1.5
    PENALTY_AREA_WIDTH: float = 4.0
    PENALTY_MARK_DISTANCE: float = 1.5
    CENTER_CIRCLE_RADIUS: float = 0.75
    LINE_WIDTH: float = 0.05
    MARGIN: float = 0.5
    
    # Derived coordinates
    HALF_LENGTH: float = 4.5
    HALF_WIDTH: float = 3.0
    GOAL_HALF_WIDTH: float = 1.3


@dataclass(frozen=True)
class RobotLimits:
    """Robot physical and operational limits."""
    # Size limits
    MAX_HEIGHT: float = 0.80         # meters
    MAX_DIAMETER: float = 0.52       # meters
    MAX_WEIGHT: float = 40.0         # kg
    
    # Motion limits
    MAX_LINEAR_VELOCITY: float = 1.5     # m/s
    MAX_ANGULAR_VELOCITY: float = 3.14   # rad/s
    MAX_LINEAR_ACCEL: float = 2.0        # m/s²
    MAX_ANGULAR_ACCEL: float = 6.28      # rad/s²
    
    # Motor limits
    MAX_MOTOR_RPM: int = 200
    MAX_MOTOR_CURRENT: float = 3.0       # Ampere
    
    # Kicker limits
    MAX_KICKER_VOLTAGE: float = 200.0    # Volts
    MIN_KICK_INTERVAL: float = 3.0       # seconds


@dataclass(frozen=True)
class WheelConfig:
    """3-wheel omni configuration."""
    COUNT: int = 3
    RADIUS: float = 0.05             # meters
    BASE_DISTANCE: float = 0.17      # meters from center
    ANGLES_DEG: Tuple[int, ...] = (0, 120, 240)
    GEAR_RATIO: int = 45
    ENCODER_PPR: int = 11            # Pulses per revolution
    ENCODER_CPR: int = 1980          # Counts per revolution


@dataclass(frozen=True)
class BallSpec:
    """Ball specifications."""
    DIAMETER: float = 0.065          # meters (6.5cm, size 1)
    RADIUS: float = 0.0325           # meters
    WEIGHT: float = 0.20             # kg (200g)


# =============================================================================
# Timing Constants
# =============================================================================

@dataclass(frozen=True)
class TimingConfig:
    """Game timing configuration in seconds."""
    HALF_DURATION: int = 600         # 10 minutes
    HALF_TIME_BREAK: int = 300       # 5 minutes
    OVERTIME_DURATION: int = 300     # 5 minutes
    KICKOFF_TIME: int = 10
    FREEKICK_TIME: int = 10
    PENALTY_TIME: int = 30
    PENALTY_DURATION: int = 30       # Penalty time-out


# =============================================================================
# Communication Constants
# =============================================================================

@dataclass(frozen=True)
class SerialConfig:
    """Default serial communication settings."""
    BAUDRATE: int = 115200
    TIMEOUT: float = 0.1
    RECONNECT_INTERVAL: float = 2.0


@dataclass(frozen=True)
class GameControllerConfig:
    """GameController network settings."""
    MULTICAST_ADDRESS: str = "224.5.23.1"
    PORT: int = 3838
    TIMEOUT: float = 2.0


# =============================================================================
# Topic/Service Names
# =============================================================================

class Topics:
    """Standard ROS 2 topic names."""
    # Sensor data
    SENSOR_DATA = "/krsbi/sensor_data"
    IMU_DATA = "/krsbi/imu"
    MOTOR_FEEDBACK = "/krsbi/motor_feedback"
    DISTANCE_SENSORS = "/krsbi/distance_sensors"
    
    # Commands
    MOTOR_COMMAND = "/krsbi/motor_command"
    CMD_VEL = "/krsbi/cmd_vel"
    
    # Vision
    BALL_POSITION = "/krsbi/vision/ball"
    DETECTED_OBJECTS = "/krsbi/vision/objects"
    FIELD_LINES = "/krsbi/vision/lines"
    
    # State
    ROBOT_STATE = "/krsbi/robot_state"
    ROBOT_POSE = "/krsbi/pose"
    GAME_STATE = "/krsbi/game_state"
    
    # Actuators
    GRIPPER_STATE = "/krsbi/gripper_state"
    KICKER_STATE = "/krsbi/kicker_state"
    
    # Diagnostics
    DIAGNOSTICS = "/diagnostics"
    SYSTEM_STATUS = "/krsbi/system_status"


class Services:
    """Standard ROS 2 service names."""
    SET_MOTOR_SPEED = "/krsbi/set_motor_speed"
    SET_GRIPPER = "/krsbi/set_gripper"
    CHARGE_KICKER = "/krsbi/charge_kicker"
    KICK = "/krsbi/kick"
    GET_ROBOT_STATE = "/krsbi/get_robot_state"
    CALIBRATE = "/krsbi/calibrate"
    SET_GAME_STATE = "/krsbi/set_game_state"


class Actions:
    """Standard ROS 2 action names."""
    MOVE_TO = "/krsbi/move_to"
    ROTATE_TO = "/krsbi/rotate_to"
    KICK_BALL = "/krsbi/kick_ball"
    GRAB_BALL = "/krsbi/grab_ball"
