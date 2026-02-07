
import numpy as np
from typing import Tuple, List

class OmniKinematics3:
    """
    3-Wheel Omni-directional Kinematics.
    
    Wheel Configuration:
    - Wheel 1: 0 deg (Front, drives in Y direction)
    - Wheel 2: 120 deg (Rear-Left)
    - Wheel 3: 240 deg (Rear-Right)
    
    Coordinate System:
    - X: Forward
    - Y: Left
    - Z: Up (Rotation positive CCW)
    
    Units:
    - Linear Velocity: m/s
    - Angular Velocity: rad/s
    - Wheel Velocity: rad/s (angular velocity of wheel)
    """
    
    def __init__(
        self, 
        wheel_radius: float = 0.05, 
        robot_radius: float = 0.17,
        wheel_angles: List[float] = [0.0, 120.0, 240.0],
        correction: List[float] = [1.0, 1.0, 1.0]
    ):
        """
        Initialize kinematics parameters.
        
        Args:
            wheel_radius: Radius of the omni wheel (meters)
            robot_radius: Distance from center to wheel contact point (meters)
            wheel_angles: Mounting angles of wheels (degrees)
            correction: Velocity correction factors [c1, c2, c3]
        """
        self.r = wheel_radius
        self.L = robot_radius
        self.angles = np.radians(wheel_angles)
        self.correction = np.array(correction)
        
        # Calculate Coupling Matrix (Body -> Wheel)
        # V_wheel = J * V_body
        # J rows are projection of motion onto wheel directions
        # Wheel direction is perpendicular to radius (angle + 90 deg)
        
        self.J = np.zeros((3, 3))
        for i in range(3):
            theta = self.angles[i]
            # Wheel drive direction is tangent (theta + 90 deg)
            drive_angle = theta + np.pi/2
            
            # W = (Vx * cos(drive) + Vy * sin(drive) + L * omega) / r
            self.J[i, 0] = np.cos(drive_angle) / self.r
            self.J[i, 1] = np.sin(drive_angle) / self.r
            self.J[i, 2] = self.L / self.r
            
        # Forward Kinematics Matrix (Pseudo-inverse)
        # V_body = J_inv * V_wheel
        self.J_inv = np.linalg.pinv(self.J)
        
    def inverse(self, vx: float, vy: float, omega: float) -> np.ndarray:
        """
        Calculate wheel velocities from body velocity.
        
        Args:
            vx: Linear velocity X (m/s)
            vy: Linear velocity Y (m/s)
            omega: Angular velocity Z (rad/s)
            
        Returns:
            Wheel angular velocities [w1, w2, w3] (rad/s)
        """
        v_body = np.array([vx, vy, omega])
        w_wheels = self.J @ v_body
        
        # Apply correction factors
        return w_wheels * self.correction

    def forward(self, w1: float, w2: float, w3: float) -> Tuple[float, float, float]:
        """
        Calculate body velocity from wheel velocities.
        
        Args:
            w1, w2, w3: Wheel angular velocities (rad/s)
            
        Returns:
            (vx, vy, omega) in m/s and rad/s
        """
        w_wheels = np.array([w1, w2, w3])
        
        # Remove correction before forward kinematics
        w_raw = w_wheels / self.correction
        
        v_body = self.J_inv @ w_raw
        
        return float(v_body[0]), float(v_body[1]), float(v_body[2])

    def get_wheel_rpm(self, w_rad_s: np.ndarray) -> np.ndarray:
        """Convert rad/s to RPM."""
        return w_rad_s * 60.0 / (2 * np.pi)

    def get_wheel_rad_s(self, rpm: np.ndarray) -> np.ndarray:
        """Convert RPM to rad/s."""
        return rpm * 2 * np.pi / 60.0
