"""Motor control subsystem for commanding robot motors via I2C."""

from turtle import speed
from typing import Optional

from structure.Subsystem import Subsystem
from Robot.subsystems.algorithms.KalmanStateEstimator import KalmanStateEstimator
from Robot.Constants import Constants
from Robot.subsystems.subsystemChildren.DAC import DAC
from Robot.subsystems.subsystemChildren.FrontWheelEncoder import FrontWheelEncoder
from Robot.subsystems.sensors.BackWheelEncoder import BackWheelEncoder
from helpers.ArenaBoundaryHelper import ArenaBoundaryManager, ArenaBoundary

import math
import time
import logging
import RPi.GPIO as GPIO
from simple_pid import PID
from Robot.MathUtil import MathUtil

logger = logging.getLogger(f"{__name__}.DriveTrain")
logger.setLevel(logging.INFO)  # Set to DEBUG for detailed output


class DriveTrain(Subsystem):
    
    def __init__(self):
        """Initialize motor control subsystem."""
        try:
            self._dac = DAC()
            self._front_encoder = FrontWheelEncoder()
            self._backwheel_forward_ssr_pin = Constants.backwheel_forward_ssr_pin
            self._backwheel_reverse_ssr_pin = Constants.backwheel_reverse_ssr_pin
            self._backwheel_power_ssr_pin = Constants.backwheel_power_ssr_pin
            self._frontwheel_power_ssr_pin = Constants.frontwheel_power_ssr_pin
            self._left_clutch_pin = Constants.left_clutch_pin
            self._right_clutch_pin = Constants.right_clutch_pin
            self._dac_backwheel_channel = Constants.dac_backwheel_channel
            self._dac_frontwheel_channel = Constants.dac_frontwheel_channel
            self._backwheel_power_scale_factor = Constants.backwheel_power_scale_factor
            self._inside_clutch_angle_threshold_rads = math.radians(10)
            
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(self._backwheel_forward_ssr_pin, GPIO.OUT)
            GPIO.setup(self._backwheel_reverse_ssr_pin, GPIO.OUT)
            GPIO.setup(self._backwheel_power_ssr_pin, GPIO.OUT)
            GPIO.setup(self._frontwheel_power_ssr_pin, GPIO.OUT)
            GPIO.setup(self._left_clutch_pin, GPIO.OUT)
            GPIO.setup(self._right_clutch_pin, GPIO.OUT)
            
            GPIO.output(self._backwheel_power_ssr_pin, GPIO.HIGH)
            GPIO.output(self._frontwheel_power_ssr_pin, GPIO.HIGH)
            GPIO.output(self._left_clutch_pin, GPIO.HIGH)
            GPIO.output(self._right_clutch_pin, GPIO.HIGH)
            
            self.shutdown = False
            self.is_reversing = False

            # Initialize motors to stopped state
            self._dac.write(self._dac_backwheel_channel, 0)
            self._dac.write(self._dac_frontwheel_channel, 0.5)  
            
            # Pitch PID controller
            # TODO Fine-tune PID parameters for better performance
            self.front_wheel_pid = PID(
                1.1, 0.1, 0.0, 
                setpoint=0,
                error_map=MathUtil.wrap_to_pi  # Wrap error to [-pi, pi]
            )
            self.output_limits = (-1, 1)  # Limit PID output to motor command range
            self.front_wheel_pid.output_limits = self.output_limits # Limit output to motor
            self.soft_limit = Constants.steering_angle_limit_rads
            self._last_backwheel_speed_cmd = 0.0
            self._last_backwheel_speed_update_time = time.monotonic()

            logger.info(f"DriveTrain initialized with SSR pins: {self._backwheel_forward_ssr_pin}, {self._backwheel_reverse_ssr_pin}, {self._backwheel_power_ssr_pin}, {self._frontwheel_power_ssr_pin}")
        
        except Exception as e:
            logger.error(f"Failed to initialize DriveTrain: {e}")
        
        self._angle = 0 # Default to straight
        self._speed = 0 # Default to stopped
    
    def _get_angle_from_pid(self, target_angle: float) -> float:
        """Calculate front wheel angle using PID controller based on target angle.
        target_angle: float in radians."""
        current_angle = self._front_encoder.get_position()
        if current_angle is None:
            logger.debug("Front encoder reading failed, defaulting to 0 degrees")
            return 0  # Default to straight if encoder fails
        
        #current_angle = max(min(current_angle, self.soft_limit), -self.soft_limit)  # Apply soft limits
        
        self.front_wheel_pid.setpoint = target_angle
        pid_speed = self.front_wheel_pid(current_angle)
        if pid_speed is None or math.isnan(pid_speed):
            logger.warning("PID output is NaN, defaulting to 0")
            pid_speed = 0.0
        pid_speed = self.apply_soft_limits(current_angle, pid_speed)
        
        logger.debug(f"PID target: {math.degrees(target_angle):.2f}°, current: {math.degrees(current_angle):.2f}°, output: {pid_speed:.2f}")
        
        if pid_speed is None or math.isnan(pid_speed):
            logger.warning("PID output is NaN, defaulting to 0")
            return 0
        return pid_speed
    
    def apply_soft_limits(self, current_angle: float, commanded_speed: float) -> float:
        """Apply soft limits to target angle."""
        if current_angle > self.soft_limit:
            logger.debug(f"Target angle {math.degrees(current_angle):.2f}° exceeds soft limit, applying limit")
            return min(commanded_speed, 0)  # Prevent moving forward if over limit
        elif current_angle < -self.soft_limit:
            logger.debug(f"Target angle {math.degrees(current_angle):.2f}° exceeds soft limit, applying limit")
            return max(commanded_speed, 0)  # Prevent moving forward if over limit
        return commanded_speed
    
    def set_speed_angle(self, speed: float, target_angle: float):
        """Set motor speed and angle.
        speed: -1.0 to 1.0 (negative for reverse)
        target_angle: -π/6 (-30 degrees) to π/6 (30 degrees) 
        """
        if self.shutdown:
            logger.warning("Attempted to set speed/angle after shutdown. Command ignored.")
            return
        
        # Permit direction changes only when measured wheel speed magnitude is near zero.
        is_speed_reversing = speed < 0
        wheel_speed_magnitude = abs(BackWheelEncoder().get_velocity())
        if (is_speed_reversing is not self.is_reversing) and (wheel_speed_magnitude > Constants.min_swap_speed_threshold):
            logger.debug(
                f"Direction swap blocked at wheel speed {wheel_speed_magnitude:.3f} m/s "
                f"(threshold {Constants.min_swap_speed_threshold:.3f} m/s). Clamping speed to 0."
            )
            speed = 0.0
        
        limited_speed = speed

        self._speed = limited_speed
        self._angle = target_angle
        
        # Update Kalman filter with commanded speed and angle for state estimation
        KalmanStateEstimator().set_rear_wheel_velocity(limited_speed * Constants.rear_motor_top_speed)
        current_angle = self._front_encoder.get_position()
        if current_angle is None:
            current_angle = target_angle  # Fallback to target angle if encoder fails
        KalmanStateEstimator().set_steering_angle(current_angle)
        
        # Apply inside clutch logic based on current steering angle to improve turning performance
        self._apply_inside_clutch_logic(current_angle)
        
        # Keep previous direction when command is effectively zero.
        # This avoids prematurely changing direction during a sign transition.
        if abs(limited_speed) > 1e-6:
            self.is_reversing = limited_speed < 0
        BackWheelEncoder().set_reversing(self.is_reversing)  # Inform encoder of current direction for correct velocity calculation
        self._set_wheel_directions(self.is_reversing)
        self._dac.write(self._dac_backwheel_channel, abs(limited_speed) * self._backwheel_power_scale_factor)
        
        # set front wheel speed using PID controller
        pid_speed = self._get_angle_from_pid(target_angle)
        write_value = MathUtil.map(pid_speed, self.output_limits[0], self.output_limits[1], 0, 1)
        self._dac.write(self._dac_frontwheel_channel, write_value)
        logger.debug(f"Set speed cmd: {speed:.2f}, limited: {limited_speed:.2f}, angle: {math.degrees(target_angle):.2f}, with speed of {pid_speed:.2f}, is_reversing: {self.is_reversing}")
    
    def _set_wheel_directions(self, is_reversing: bool):
        """Set SSRs for wheel direction based on speed sign."""
        try:
            if not is_reversing:
                # Forward
                GPIO.output(self._backwheel_forward_ssr_pin, GPIO.HIGH)
                GPIO.output(self._backwheel_reverse_ssr_pin, GPIO.LOW)
            else:
                # Reverse
                GPIO.output(self._backwheel_forward_ssr_pin, GPIO.LOW)
                GPIO.output(self._backwheel_reverse_ssr_pin, GPIO.HIGH)
        except Exception as e:
            logger.error(f"Error setting wheel directions: {e}")
            
    def disengage_frontwheel(self):
        try:
            GPIO.output(self._frontwheel_power_ssr_pin, GPIO.LOW)
        except Exception as e:
            logger.error(f"Failed to disengage front wheel: {e}")

    def engage_frontwheel(self):
        try:
            GPIO.output(self._frontwheel_power_ssr_pin, GPIO.HIGH)
        except Exception as e:
            logger.error(f"Failed to engage front wheel: {e}")
        
    def disengage_backwheel(self):
        try:
            GPIO.output(self._backwheel_power_ssr_pin, GPIO.LOW)
        except Exception as e:
            logger.error(f"Failed to disengage back wheel: {e}")
        
    def engage_backwheel(self):
        try:
            GPIO.output(self._backwheel_power_ssr_pin, GPIO.HIGH)
        except Exception as e:
            logger.error(f"Failed to engage back wheel: {e}")

    def get_current_commands(self):
        """Get current motor commands.
        
        Returns:
            tuple: (speed, angle)
        """
        return self._speed, self._angle
    
    def get_frontwheel_position(self) -> Optional[float]:
        """Get current front wheel encoder position.
        
        Returns:
            float: Current position from encoder or None on error
        """
        return self._front_encoder.get_position()
    
    def is_approaching_boundary(self, speed: float) -> bool:
        """Determines if the robot needs to stop based on its current power speed [-1, -1] and distance from the arena boundary.

        Args:
            speed (float): Current commanded speed, used to calculate stopping distance. Expected to be in range [-1, 1], where 1 corresponds to max speed defined in Constants.

        Returns:
            bool: True if the robot should stop to avoid crossing the boundary, False otherwise. If boundary information is unavailable, returns True to be safe.
        """
        Constants().distance_to_stop_from_full_speed
        distance_from_boundary = self.get_distance_from_boundary()
        if distance_from_boundary == -1.0:
            logger.debug("Boundary information unavailable, treating as if at boundary to be safe")
            return False  # Boundary not set, ignore boundary limit (might want to return True to be safe)
        
        # Calculate stopping distance based on current speed. Scale speed to actual velocity using rear_motor_top_speed.
        stopping_distance = abs(speed) * (Constants().distance_to_stop_from_full_speed)
        logger.debug(f"Distance from boundary: {distance_from_boundary:.2f} m, stopping distance at current speed: {stopping_distance:.2f} m")
        
        return stopping_distance >= distance_from_boundary
        
    
    def get_distance_from_boundary(self) -> float:
        """Get distance from the nearest arena boundary.
        
        Returns:
            float: Distance in meters or 0 if not found
        """
        boundary = ArenaBoundaryManager().get_arena_boundary()
        if boundary is None:
            logger.warning("No arena boundary found, returning distance of -1 (ignore)")
            return -1.0
        current_position = KalmanStateEstimator().get_robot_pose()
        if current_position is None:
            logger.warning("Current robot position unknown, returning distance of 0")
            return 0.0
        distance = boundary.shortest_distance_to_edge(current_position[0], current_position[1])
        logger.debug(f"Distance from boundary: {distance:.3f} meters")
        
        return distance
    
    def stop(self):
        """Stop motors (set speed and angle to zero)."""
        KalmanStateEstimator().set_rear_wheel_velocity(0.0)
        
        self._last_backwheel_speed_cmd = 0.0
        self._last_backwheel_speed_update_time = time.monotonic()
        self._speed = 0.0
        self._dac.write(self._dac_backwheel_channel, 0.0)
        self._dac.write(self._dac_frontwheel_channel, 0.5)
        self._set_left_clutch(True)
        self._set_right_clutch(True)

    def _set_left_clutch(self, engaged: bool):
        GPIO.output(self._left_clutch_pin, GPIO.HIGH if engaged else GPIO.LOW)

    def _set_right_clutch(self, engaged: bool):
        GPIO.output(self._right_clutch_pin, GPIO.HIGH if engaged else GPIO.LOW)

    def _apply_inside_clutch_logic(self, steering_angle_rads: float):
        """Disengage inside rear clutch when steering exceeds threshold."""
        if steering_angle_rads > self._inside_clutch_angle_threshold_rads:
            # Left turn: left wheel is inside turn radius.
            self._set_left_clutch(False)
            self._set_right_clutch(True)
        elif steering_angle_rads < -self._inside_clutch_angle_threshold_rads:
            # Right turn: right wheel is inside turn radius.
            self._set_left_clutch(True)
            self._set_right_clutch(False)
        else:
            # Near straight steering, keep both rear clutches engaged.
            self._set_left_clutch(True)
            self._set_right_clutch(True)
    
    def reset_pid(self):
        """Reset the front wheel PID controller.
        
        Clears integral and derivative terms to reset controller state.
        Useful when starting new maneuvers or recovering from disturbances.
        """
        self.front_wheel_pid.reset()
        logger.info("Front wheel PID controller reset")
    
    def close(self):
        if not self.shutdown:
            try:
                self.shutdown = True
                self._dac.close()
                self._front_encoder.close()
                GPIO.cleanup(self._backwheel_forward_ssr_pin)
                GPIO.cleanup(self._backwheel_reverse_ssr_pin)
                GPIO.cleanup(self._backwheel_power_ssr_pin)
                GPIO.cleanup(self._frontwheel_power_ssr_pin)
                GPIO.cleanup(self._left_clutch_pin)
                GPIO.cleanup(self._right_clutch_pin)
            except Exception:
                pass
                        

