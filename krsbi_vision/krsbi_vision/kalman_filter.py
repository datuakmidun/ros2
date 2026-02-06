"""
KRSBI-B Soccer Robot - Kalman Filter

Kalman filter implementations for ball and robot tracking.
Supports constant velocity and constant acceleration models.
"""

import numpy as np
from typing import Optional, Tuple, List
from dataclasses import dataclass, field
import time


@dataclass
class TrackState:
    """Track state for Kalman filter."""
    id: int
    state: np.ndarray              # State vector
    covariance: np.ndarray         # Covariance matrix
    age: int = 0                   # Frames since creation
    hits: int = 0                  # Total detections
    time_since_update: int = 0     # Frames since last detection
    confidence: float = 1.0        # Track confidence [0, 1]
    class_id: int = 0              # Object class
    is_confirmed: bool = False     # Track confirmed
    history: List[np.ndarray] = field(default_factory=list)


class KalmanFilter2D:
    """
    2D Kalman Filter with configurable motion model.
    
    Supports:
    - Constant Velocity (CV) model: state = [x, y, vx, vy]
    - Constant Acceleration (CA) model: state = [x, y, vx, vy, ax, ay]
    """
    
    def __init__(
        self,
        model: str = "constant_velocity",
        dt: float = 0.02,
        process_noise_pos: float = 0.1,
        process_noise_vel: float = 1.0,
        process_noise_acc: float = 5.0,
        measurement_noise: float = 2.0,
    ):
        """
        Initialize Kalman filter.
        
        Args:
            model: Motion model ("constant_velocity" or "constant_acceleration")
            dt: Time step (seconds)
            process_noise_pos: Process noise for position
            process_noise_vel: Process noise for velocity
            process_noise_acc: Process noise for acceleration
            measurement_noise: Measurement noise for position
        """
        self.model = model
        self.dt = dt
        
        if model == "constant_acceleration":
            # State: [x, y, vx, vy, ax, ay]
            self.dim_x = 6
            self.dim_z = 2
            
            # State transition matrix F
            self.F = np.array([
                [1, 0, dt, 0, 0.5*dt**2, 0],
                [0, 1, 0, dt, 0, 0.5*dt**2],
                [0, 0, 1, 0, dt, 0],
                [0, 0, 0, 1, 0, dt],
                [0, 0, 0, 0, 1, 0],
                [0, 0, 0, 0, 0, 1],
            ])
            
            # Process noise Q
            self.Q = np.diag([
                process_noise_pos, process_noise_pos,
                process_noise_vel, process_noise_vel,
                process_noise_acc, process_noise_acc,
            ])
            
            # Initial covariance
            self._P_init = np.diag([100, 100, 50, 50, 10, 10])
            
        else:  # constant_velocity
            # State: [x, y, vx, vy]
            self.dim_x = 4
            self.dim_z = 2
            
            # State transition matrix F
            self.F = np.array([
                [1, 0, dt, 0],
                [0, 1, 0, dt],
                [0, 0, 1, 0],
                [0, 0, 0, 1],
            ])
            
            # Process noise Q
            self.Q = np.diag([
                process_noise_pos, process_noise_pos,
                process_noise_vel, process_noise_vel,
            ])
            
            # Initial covariance
            self._P_init = np.diag([100, 100, 50, 50])
        
        # Measurement matrix H (observe position only)
        self.H = np.zeros((2, self.dim_x))
        self.H[0, 0] = 1  # x
        self.H[1, 1] = 1  # y
        
        # Measurement noise R
        self.R = np.diag([measurement_noise, measurement_noise])
        
        # Identity matrix
        self.I = np.eye(self.dim_x)
    
    def initiate(self, measurement: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        Initialize state from first measurement.
        
        Args:
            measurement: [x, y] position
            
        Returns:
            (state, covariance) tuple
        """
        state = np.zeros(self.dim_x)
        state[0] = measurement[0]  # x
        state[1] = measurement[1]  # y
        # Velocities and accelerations initialized to 0
        
        covariance = self._P_init.copy()
        
        return state, covariance
    
    def predict(
        self, 
        state: np.ndarray, 
        covariance: np.ndarray,
        dt: Optional[float] = None,
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        Predict next state.
        
        Args:
            state: Current state vector
            covariance: Current covariance matrix
            dt: Optional custom time step
            
        Returns:
            (predicted_state, predicted_covariance) tuple
        """
        if dt is not None and dt != self.dt:
            # Update F matrix for custom dt
            F = self._get_F(dt)
        else:
            F = self.F
        
        # State prediction
        predicted_state = F @ state
        
        # Covariance prediction
        predicted_covariance = F @ covariance @ F.T + self.Q
        
        return predicted_state, predicted_covariance
    
    def update(
        self,
        state: np.ndarray,
        covariance: np.ndarray,
        measurement: np.ndarray,
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        Update state with measurement.
        
        Args:
            state: Predicted state vector
            covariance: Predicted covariance matrix
            measurement: [x, y] measurement
            
        Returns:
            (updated_state, updated_covariance) tuple
        """
        # Innovation (measurement residual)
        y = measurement - self.H @ state
        
        # Innovation covariance
        S = self.H @ covariance @ self.H.T + self.R
        
        # Kalman gain
        K = covariance @ self.H.T @ np.linalg.inv(S)
        
        # State update
        updated_state = state + K @ y
        
        # Covariance update (Joseph form for numerical stability)
        temp = self.I - K @ self.H
        updated_covariance = temp @ covariance @ temp.T + K @ self.R @ K.T
        
        return updated_state, updated_covariance
    
    def gating_distance(
        self,
        state: np.ndarray,
        covariance: np.ndarray,
        measurement: np.ndarray,
        only_position: bool = True,
    ) -> float:
        """
        Compute Mahalanobis distance for gating.
        
        Args:
            state: State vector
            covariance: Covariance matrix
            measurement: Measurement vector
            only_position: Use only position for distance
            
        Returns:
            Mahalanobis distance
        """
        predicted_measurement = self.H @ state
        residual = measurement - predicted_measurement
        
        # Innovation covariance
        S = self.H @ covariance @ self.H.T + self.R
        
        # Mahalanobis distance
        try:
            S_inv = np.linalg.inv(S)
            distance = np.sqrt(residual.T @ S_inv @ residual)
        except np.linalg.LinAlgError:
            distance = float('inf')
        
        return distance
    
    def _get_F(self, dt: float) -> np.ndarray:
        """Get state transition matrix for given dt."""
        if self.model == "constant_acceleration":
            return np.array([
                [1, 0, dt, 0, 0.5*dt**2, 0],
                [0, 1, 0, dt, 0, 0.5*dt**2],
                [0, 0, 1, 0, dt, 0],
                [0, 0, 0, 1, 0, dt],
                [0, 0, 0, 0, 1, 0],
                [0, 0, 0, 0, 0, 1],
            ])
        else:
            return np.array([
                [1, 0, dt, 0],
                [0, 1, 0, dt],
                [0, 0, 1, 0],
                [0, 0, 0, 1],
            ])
    
    def get_position(self, state: np.ndarray) -> np.ndarray:
        """Extract position from state."""
        return state[:2]
    
    def get_velocity(self, state: np.ndarray) -> np.ndarray:
        """Extract velocity from state."""
        return state[2:4]
    
    def get_acceleration(self, state: np.ndarray) -> Optional[np.ndarray]:
        """Extract acceleration from state (CA model only)."""
        if self.model == "constant_acceleration":
            return state[4:6]
        return None


class BallTracker:
    """
    Ball tracker using Kalman filter with physics model.
    
    Tracks single ball with:
    - Constant acceleration model
    - Physics-based prediction (friction)
    - Confidence decay when lost
    """
    
    def __init__(
        self,
        dt: float = 0.02,
        process_noise_pos: float = 0.1,
        process_noise_vel: float = 1.0,
        process_noise_acc: float = 5.0,
        measurement_noise: float = 2.0,
        max_age: int = 30,
        min_hits: int = 3,
        friction: float = 0.3,
        max_velocity: float = 3.0,
    ):
        """
        Initialize ball tracker.
        
        Args:
            dt: Time step
            process_noise_*: Kalman filter process noise
            measurement_noise: Kalman filter measurement noise
            max_age: Max frames without detection before lost
            min_hits: Minimum detections to confirm track
            friction: Ball friction coefficient
            max_velocity: Maximum ball velocity (m/s)
        """
        self.kf = KalmanFilter2D(
            model="constant_acceleration",
            dt=dt,
            process_noise_pos=process_noise_pos,
            process_noise_vel=process_noise_vel,
            process_noise_acc=process_noise_acc,
            measurement_noise=measurement_noise,
        )
        
        self.dt = dt
        self.max_age = max_age
        self.min_hits = min_hits
        self.friction = friction
        self.max_velocity = max_velocity
        
        # Track state
        self.track: Optional[TrackState] = None
        self.last_time = time.time()
    
    def update(
        self,
        detection: Optional[np.ndarray] = None,
        timestamp: Optional[float] = None,
    ) -> Optional[TrackState]:
        """
        Update tracker with detection (or None if not detected).
        
        Args:
            detection: [x, y] ball position or None
            timestamp: Detection timestamp
            
        Returns:
            Current track state or None
        """
        # Calculate dt
        current_time = timestamp if timestamp else time.time()
        dt = current_time - self.last_time
        self.last_time = current_time
        
        if self.track is None:
            # No existing track
            if detection is not None:
                # Initialize new track
                state, covariance = self.kf.initiate(detection)
                self.track = TrackState(
                    id=1,
                    state=state,
                    covariance=covariance,
                    hits=1,
                )
                self.track.history.append(detection.copy())
        else:
            # Predict
            self.track.state, self.track.covariance = self.kf.predict(
                self.track.state, self.track.covariance, dt
            )
            
            # Apply physics (friction)
            self._apply_physics(dt)
            
            if detection is not None:
                # Update with detection
                self.track.state, self.track.covariance = self.kf.update(
                    self.track.state, self.track.covariance, detection
                )
                self.track.hits += 1
                self.track.time_since_update = 0
                self.track.confidence = min(1.0, self.track.confidence + 0.2)
                self.track.history.append(detection.copy())
                
                # Confirm track
                if self.track.hits >= self.min_hits:
                    self.track.is_confirmed = True
            else:
                # No detection
                self.track.time_since_update += 1
                self.track.confidence = max(0.0, self.track.confidence - 0.1)
            
            self.track.age += 1
            
            # Delete old tracks
            if self.track.time_since_update > self.max_age:
                self.track = None
            
            # Limit history
            if self.track and len(self.track.history) > 50:
                self.track.history = self.track.history[-50:]
        
        return self.track
    
    def _apply_physics(self, dt: float):
        """Apply physics model (friction) to state."""
        if self.track is None:
            return
        
        # Get velocity
        vx, vy = self.kf.get_velocity(self.track.state)
        speed = np.sqrt(vx**2 + vy**2)
        
        if speed > 0.01:
            # Apply friction deceleration
            friction_accel = self.friction * 9.81  # a = μg
            new_speed = max(0, speed - friction_accel * dt)
            
            # Clamp to max velocity
            new_speed = min(new_speed, self.max_velocity)
            
            # Update velocity
            scale = new_speed / speed
            self.track.state[2] *= scale  # vx
            self.track.state[3] *= scale  # vy
    
    def predict_trajectory(
        self,
        duration: float = 1.0,
        step: float = 0.02,
    ) -> List[np.ndarray]:
        """
        Predict future ball trajectory.
        
        Args:
            duration: Prediction duration (seconds)
            step: Time step (seconds)
            
        Returns:
            List of [x, y] positions
        """
        if self.track is None:
            return []
        
        trajectory = []
        state = self.track.state.copy()
        covariance = self.track.covariance.copy()
        
        t = 0
        while t < duration:
            # Predict
            state, covariance = self.kf.predict(state, covariance, step)
            
            # Apply friction
            vx, vy = state[2], state[3]
            speed = np.sqrt(vx**2 + vy**2)
            if speed > 0.01:
                friction_accel = self.friction * 9.81
                new_speed = max(0, speed - friction_accel * step)
                scale = new_speed / speed if speed > 0 else 0
                state[2] *= scale
                state[3] *= scale
            
            trajectory.append(state[:2].copy())
            t += step
        
        return trajectory
    
    def get_position(self) -> Optional[np.ndarray]:
        """Get current ball position."""
        if self.track is None:
            return None
        return self.kf.get_position(self.track.state)
    
    def get_velocity(self) -> Optional[np.ndarray]:
        """Get current ball velocity."""
        if self.track is None:
            return None
        return self.kf.get_velocity(self.track.state)
    
    def is_tracking(self) -> bool:
        """Check if actively tracking."""
        return self.track is not None and self.track.is_confirmed
    
    def reset(self):
        """Reset tracker."""
        self.track = None


class MultiObjectTracker:
    """
    Multi-object tracker using Kalman filter and Hungarian algorithm.
    
    For tracking multiple robots.
    """
    
    def __init__(
        self,
        dt: float = 0.02,
        max_age: int = 50,
        min_hits: int = 5,
        distance_threshold: float = 150.0,
        use_mahalanobis: bool = True,
        mahalanobis_threshold: float = 11.07,
    ):
        """
        Initialize multi-object tracker.
        
        Args:
            dt: Time step
            max_age: Max frames without detection
            min_hits: Min detections to confirm
            distance_threshold: Max Euclidean distance for association
            use_mahalanobis: Use Mahalanobis distance
            mahalanobis_threshold: Threshold for Mahalanobis gating
        """
        self.kf = KalmanFilter2D(model="constant_velocity", dt=dt)
        self.dt = dt
        self.max_age = max_age
        self.min_hits = min_hits
        self.distance_threshold = distance_threshold
        self.use_mahalanobis = use_mahalanobis
        self.mahalanobis_threshold = mahalanobis_threshold
        
        self.tracks: List[TrackState] = []
        self.next_id = 1
    
    def update(self, detections: List[np.ndarray]) -> List[TrackState]:
        """
        Update tracker with detections.
        
        Args:
            detections: List of [x, y] positions
            
        Returns:
            List of confirmed tracks
        """
        # Predict all tracks
        for track in self.tracks:
            track.state, track.covariance = self.kf.predict(
                track.state, track.covariance
            )
            track.age += 1
            track.time_since_update += 1
        
        if len(detections) == 0:
            # No detections, just predict
            self._manage_tracks()
            return self._get_confirmed_tracks()
        
        if len(self.tracks) == 0:
            # No tracks, create from detections
            for det in detections:
                self._create_track(det)
            return self._get_confirmed_tracks()
        
        # Build cost matrix
        cost_matrix = self._compute_cost_matrix(detections)
        
        # Hungarian algorithm
        matched, unmatched_tracks, unmatched_dets = self._hungarian_match(
            cost_matrix, len(self.tracks), len(detections)
        )
        
        # Update matched tracks
        for track_idx, det_idx in matched:
            track = self.tracks[track_idx]
            det = np.array(detections[det_idx])
            track.state, track.covariance = self.kf.update(
                track.state, track.covariance, det
            )
            track.hits += 1
            track.time_since_update = 0
            if track.hits >= self.min_hits:
                track.is_confirmed = True
        
        # Create new tracks for unmatched detections
        for det_idx in unmatched_dets:
            self._create_track(detections[det_idx])
        
        # Manage tracks
        self._manage_tracks()
        
        return self._get_confirmed_tracks()
    
    def _compute_cost_matrix(self, detections: List[np.ndarray]) -> np.ndarray:
        """Compute cost matrix for assignment."""
        n_tracks = len(self.tracks)
        n_dets = len(detections)
        cost = np.zeros((n_tracks, n_dets))
        
        for i, track in enumerate(self.tracks):
            for j, det in enumerate(detections):
                det_arr = np.array(det)
                if self.use_mahalanobis:
                    dist = self.kf.gating_distance(
                        track.state, track.covariance, det_arr
                    )
                    if dist > self.mahalanobis_threshold:
                        cost[i, j] = 1e6  # High cost for gating
                    else:
                        cost[i, j] = dist
                else:
                    pos = self.kf.get_position(track.state)
                    dist = np.linalg.norm(pos - det_arr)
                    cost[i, j] = dist
        
        return cost
    
    def _hungarian_match(
        self,
        cost_matrix: np.ndarray,
        n_tracks: int,
        n_dets: int,
    ) -> Tuple[List[Tuple[int, int]], List[int], List[int]]:
        """Hungarian algorithm matching."""
        try:
            from scipy.optimize import linear_sum_assignment
            row_ind, col_ind = linear_sum_assignment(cost_matrix)
            
            matched = []
            unmatched_tracks = list(range(n_tracks))
            unmatched_dets = list(range(n_dets))
            
            for r, c in zip(row_ind, col_ind):
                if cost_matrix[r, c] < self.distance_threshold:
                    matched.append((r, c))
                    unmatched_tracks.remove(r)
                    unmatched_dets.remove(c)
            
            return matched, unmatched_tracks, unmatched_dets
            
        except ImportError:
            # Fallback to greedy matching
            return self._greedy_match(cost_matrix, n_tracks, n_dets)
    
    def _greedy_match(
        self,
        cost_matrix: np.ndarray,
        n_tracks: int,
        n_dets: int,
    ) -> Tuple[List[Tuple[int, int]], List[int], List[int]]:
        """Greedy matching fallback."""
        matched = []
        unmatched_tracks = list(range(n_tracks))
        unmatched_dets = list(range(n_dets))
        
        while len(unmatched_tracks) > 0 and len(unmatched_dets) > 0:
            min_cost = float('inf')
            min_t, min_d = -1, -1
            
            for t in unmatched_tracks:
                for d in unmatched_dets:
                    if cost_matrix[t, d] < min_cost:
                        min_cost = cost_matrix[t, d]
                        min_t, min_d = t, d
            
            if min_cost < self.distance_threshold:
                matched.append((min_t, min_d))
                unmatched_tracks.remove(min_t)
                unmatched_dets.remove(min_d)
            else:
                break
        
        return matched, unmatched_tracks, unmatched_dets
    
    def _create_track(self, detection: np.ndarray):
        """Create new track from detection."""
        state, covariance = self.kf.initiate(np.array(detection))
        track = TrackState(
            id=self.next_id,
            state=state,
            covariance=covariance,
            hits=1,
        )
        self.tracks.append(track)
        self.next_id += 1
    
    def _manage_tracks(self):
        """Remove old tracks."""
        self.tracks = [
            t for t in self.tracks
            if t.time_since_update <= self.max_age
        ]
    
    def _get_confirmed_tracks(self) -> List[TrackState]:
        """Get confirmed tracks."""
        return [t for t in self.tracks if t.is_confirmed]
    
    def reset(self):
        """Reset all tracks."""
        self.tracks.clear()
        self.next_id = 1
