from dataclasses import dataclass, field
from typing import List, Optional, Tuple
import numpy as np

@dataclass
class RobotPose:
    x: float
    y: float
    theta: float
    confidence: float = 1.0

@dataclass
class BallState:
    x: float
    y: float
    vx: float = 0.0
    vy: float = 0.0
    is_visible: bool = True
    last_seen: float = 0.0

@dataclass
class WorldState:
    """
    Represents the complete state of the world from the robot's perspective.
    Coordinate system:
    - X: Forward (towards opponent goal)
    - Y: Left
    - (0,0): Center of field
    """
    
    # Self state
    robot_pose: RobotPose = field(default_factory=lambda: RobotPose(0, 0, 0))

    # Ball
    ball: Optional[BallState] = None

    # Team
    teammates: List[RobotPose] = field(default_factory=list)

    # Opponents
    opponents: List[RobotPose] = field(default_factory=list)

    # Game state
    game_phase: str = "INITIAL"  # INITIAL, READY, SET, PLAYING, FINISHED
    own_score: int = 0
    opponent_score: int = 0
    
    # Strategy
    current_role: str = "Unassigned"

    # Field info (Dimensions in meters)
    field_length: float = 12.0
    field_width: float = 8.0
    own_goal_pos: Tuple[float, float] = (-6.0, 0.0)
    opponent_goal_pos: Tuple[float, float] = (6.0, 0.0)

    def ball_distance(self) -> float:
        if self.ball is None:
            return float('inf')
        # Here we assume ball position is GLOBAL.
        # krsbi_vision publishes LOCAL ball position (relative to robot).
        # Need to handle conversion or storage convention.
        # If stored as Global:
        dx = self.ball.x - self.robot_pose.x
        dy = self.ball.y - self.robot_pose.y
        return np.sqrt(dx**2 + dy**2)

    def ball_angle(self) -> float:
        if self.ball is None:
            return 0.0
        dx = self.ball.x - self.robot_pose.x
        dy = self.ball.y - self.robot_pose.y
        return np.arctan2(dy, dx) - self.robot_pose.theta
    
    def is_ball_kickable(self, distance_threshold: float = 0.3, angle_threshold: float = 0.1) -> bool:
        if not self.ball or not self.ball.is_visible:
            return False
        
        # If BallState is global, we compute relative.
        # If BallState is local (relative), check magnitude.
        # Assuming Global storage for WorldModel consistency.
        dist = self.ball_distance()
        angle = self.ball_angle()
        
        # Normalize angle
        while angle > np.pi: angle -= 2*np.pi
        while angle < -np.pi: angle += 2*np.pi
        
        return dist < distance_threshold and abs(angle) < angle_threshold
