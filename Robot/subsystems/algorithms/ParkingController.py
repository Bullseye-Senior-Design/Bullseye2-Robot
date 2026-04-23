import math
import numpy as np
from Robot.Constants import Constants
from typing import Tuple

class ParkingController:
    """Lyapunov-based kinematic controller for precise goal alignment."""
    
    def __init__(self):
        # Kinematic parameters
        self.L = Constants.wheel_base_width
        self.max_v = 0.5 # IMPORTANT: Limit max velocity for better stability during parking
        self.min_v = 0.1  # Minimum normalized drive command to keep the drivetrain moving through deadband.
        self.max_delta = Constants.steering_angle_limit_rads
        
        # Controller Gains (TUNE THESE)
        # k_rho: How aggressively to close the distance.
        # k_alpha: How aggressively to point the nose at the target.
        # k_beta: How aggressively to align with the final desired heading.
        self.k_rho = 0.5   
        self.k_alpha = 1.5 
        self.k_beta = -0.5 # Usually negative in Astolfi/Lyapunov proofs
        
    def compute_commands(self, current_pose, goal_pose) -> Tuple[float, float]:
        """
        Calculates Velocity and Steering Angle to park at the goal.
        
        Args:
            current_pose: [x, y, theta]
            goal_pose: [x, y, theta]
            
        Returns:
            v_cmd (float), delta_cmd (float)
        """
        x, y, th = current_pose
        gx, gy, gth = goal_pose
        
        # 1. Calculate Cartesian Errors
        dx = gx - x
        dy = gy - y
        rho = math.hypot(dx, dy)
        
        # Stop if we are practically there (Tolerance)
        if rho < 0.05:
            return 0.0, 0.0
            
        # 2. Calculate Polar Errors
        angle_to_goal = math.atan2(dy, dx)
        alpha = self.normalize_angle(angle_to_goal - th)
        beta = self.normalize_angle(gth - angle_to_goal)
        
        # 3. Handle Reverse Gear dynamically
        # If the goal is behind the robot (alpha > 90 deg), drive in reverse!
        direction = 1.0
        if abs(alpha) > (math.pi / 2.0):
            direction = -1.0
            alpha = self.normalize_angle(alpha - math.pi)
            beta = self.normalize_angle(beta - math.pi)
            
        # 4. Lyapunov Control Law
        # Command linear velocity (v)
        v_cmd = direction * self.k_rho * rho

        # Keep moving while we still need to correct yaw; otherwise the drivetrain can stall
        # before the heading error is resolved.
        if abs(v_cmd) < self.min_v:
            v_cmd = direction * self.min_v
        
        # Command angular velocity (omega)
        w_cmd = self.k_alpha * alpha + self.k_beta * beta
        
        # 5. Convert Unicycle Omega to Ackermann Steering Angle (Delta)
        # Positive delta means left steering and negative delta means right steering,
        # which matches the drivetrain's convention.
        # Formula: w = (v / L) * tan(delta)  -->  delta = atan(w * L / v)
        if abs(v_cmd) > 0.01:
            delta_cmd = math.atan((w_cmd * self.L) / v_cmd)
        else:
            # If velocity is near zero, just point wheels where omega wants to go
            delta_cmd = np.sign(w_cmd) * self.max_delta 
            
        # 6. Apply physical limits (Clamp)
        v_cmd = np.clip(v_cmd, -self.max_v, self.max_v)
        delta_cmd = np.clip(delta_cmd, -self.max_delta, self.max_delta)
        
        return v_cmd, delta_cmd
    
    def normalize_angle(self, angle) -> float:
        """Keep angle between -pi and pi."""
        return (angle + np.pi) % (2 * np.pi) - np.pi
    
    def is_at_goal(self, current_pose, goal_pose, pos_threshold: float, angle_threshold_rads: float) -> bool:
        """Check if we are close enough to the goal pose."""
        x, y, th = current_pose
        gx, gy, gth = goal_pose
        
        # Position error
        pos_error = math.hypot(gx - x, gy - y)
        
        # Angle error
        angle_error = abs(self.normalize_angle(gth - th))
        
        return pos_error < pos_threshold and angle_error < angle_threshold_rads