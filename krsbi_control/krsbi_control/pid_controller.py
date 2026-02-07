
import time

class PIDController:
    """
    Discrete PID Controller with integral windup protection and feed-forward.
    """
    
    def __init__(
        self,
        kp: float = 1.0, 
        ki: float = 0.0, 
        kd: float = 0.0, 
        k_ff: float = 0.0,
        output_limits: tuple = (-100, 100),
        integral_limit: float = 100.0
    ):
        """
        Initialize PID parameters.
        
        Args:
            kp: Proportional gain
            ki: Integral gain
            kd: Derivative gain
            k_ff: Feed-forward gain
            output_limits: (min, max) output range
            integral_limit: Max integral term value
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.k_ff = k_ff
        self.min_out, self.max_out = output_limits
        self.integral_limit = integral_limit
        
        self.reset()

    def reset(self):
        """Reset internal state."""
        self._integral = 0.0
        self._prev_error = 0.0
        self._last_time = time.time()

    def update(self, setpoint: float, measurement: float, dt: float = None) -> float:
        """
        Calculate control output.
        
        Args:
            setpoint: Desired value
            measurement: Process variable
            dt: Time step (seconds). If None, calculated from last update.
            
        Returns:
            Control output within limits.
        """
        current_time = time.time()
        
        if dt is None:
            dt = current_time - self._last_time
            if dt <= 0:
                dt = 1e-3  # Avoid division by zero
        
        error = setpoint - measurement
        
        # Proportional term
        p_term = self.kp * error
        
        # Integral term with clamping
        self._integral += error * dt
        self._integral = max(min(self._integral, self.integral_limit), -self.integral_limit)
        i_term = self.ki * self._integral
        
        # Derivative term
        derivative = (error - self._prev_error) / dt
        d_term = self.kd * derivative
        
        # Feed-forward term if applicable
        ff_term = self.k_ff * setpoint
        
        # Total output
        output = p_term + i_term + d_term + ff_term
        
        # Clamp output
        output = max(min(output, self.max_out), self.min_out)
        
        # Update state
        self._prev_error = error
        self._last_time = current_time
        
        return output

    @property
    def integral(self):
        return self._integral
