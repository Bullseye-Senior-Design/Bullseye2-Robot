"""Motor control subsystem for commanding robot motors via I2C."""

from turtle import speed
from typing import Optional

from structure.Subsystem import Subsystem
from Robot.subsystems.KalmanStateEstimator import KalmanStateEstimator
from Robot.Constants import Constants
from Robot.subsystems.subsystemChildren.DAC import DAC
from Robot.subsystems.subsystemChildren.FrontWheelEncoder import FrontWheelEncoder

import math
import logging
import spidev
import RPi.GPIO as GPIO

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
            self._dac_backwheel_channel = Constants.dac_backwheel_channel
            self._dac_frontwheel_channel = Constants.dac_frontwheel_channel
            self._frontwheel_power_scale_factor = Constants.frontwheel_power_scale_factor
            self._backwheel_power_scale_factor = Constants.backwheel_power_scale_factor
            
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(self._backwheel_forward_ssr_pin, GPIO.OUT)
            GPIO.setup(self._backwheel_reverse_ssr_pin, GPIO.OUT)
            GPIO.setup(self._backwheel_power_ssr_pin, GPIO.OUT)
            GPIO.setup(self._frontwheel_power_ssr_pin, GPIO.OUT)
            
            GPIO.output(self._backwheel_power_ssr_pin, GPIO.HIGH)
            GPIO.output(self._frontwheel_power_ssr_pin, GPIO.HIGH)
            self.shutdown = False

            self._dac.write_dac(self._dac_backwheel_channel, 0)
            self._dac.write_dac(self._dac_frontwheel_channel, 0.5)  # Simple mapping for demonstration

            logger.info(f"DriveTrain initialized with SSR pins: {self._backwheel_forward_ssr_pin}, {self._backwheel_reverse_ssr_pin}, {self._backwheel_power_ssr_pin}, {self._frontwheel_power_ssr_pin}")
        
        except Exception as e:
            logger.error(f"Failed to initialize DriveTrain: {e}")
        
        self._angle = 90 # Default to straight
        self._speed = 0 # Default to stopped
    
    def set_speed_angle(self, speed: float, angle: float):
        """Set motor speed and angle.
        speed: -1.0 to 1.0 (negative for reverse)
        angle: 0 to 180 (90 is straight, <90 left, >90 right)
        """
        if self.shutdown:
            logger.warning("Attempted to set speed/angle after shutdown. Command ignored.")
            return
        
        self._speed = speed
        self._angle = angle

        is_reverse = speed < 0
        self._set_wheel_directions(is_reverse)
        #self._dac.write_dac(self._dac_backwheel_channel, abs(speed) * self._backwheel_power_scale_factor)
        #self._dac.write_dac(self._dac_frontwheel_channel, abs(angle - 90) / 90 * self._frontwheel_power_scale_factor)  # Simple mapping for demonstration
        logger.debug(f"Set speed: {speed}, angle: {angle}, is_reverse: {is_reverse}")

    def _set_wheel_directions(self, is_reverse: bool):
        """Set SSRs for wheel direction based on speed sign."""
        try:
            if not is_reverse:
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
    
    def stop(self):
        """Stop motors (set speed and angle to zero)."""
        self.set_speed_angle(0, 0)
    
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
            except Exception:
                pass
                        

