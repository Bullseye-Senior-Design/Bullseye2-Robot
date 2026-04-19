import logging

import RPi.GPIO as GPIO

from Robot.Constants import Constants
from structure.Subsystem import Subsystem
logger = logging.getLogger(f"{__name__}.Clutches")
logger.setLevel(logging.INFO)  # Set to INFO for detailed output

class Clutches(Subsystem):
    
    def __init__(self):
        """Initialize clutch control GPIO pins and state."""
        super().__init__()
        # GPIO pin setup code here
        self.left_clutch_pin = Constants.left_clutch_pin
        self.right_clutch_pin = Constants.right_clutch_pin
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.left_clutch_pin, GPIO.OUT)
        GPIO.setup(self.right_clutch_pin, GPIO.OUT)

    def engage_clutches(self):
        """Engage both clutches."""
        logger.debug("Engaging both clutches")
        self.engage_left_clutch()
        self.engage_right_clutch()

    def disengage_clutches(self):
        """Disengage both clutches."""
        logger.debug("Disengaging both clutches")
        self.disengage_left_clutch()
        self.disengage_right_clutch()
    
    def engage_left_clutch(self):
        """Engage the left clutch."""
        try:
            GPIO.output(self.left_clutch_pin, GPIO.HIGH)
        except Exception as e:
            logger.error(f"Failed to engage left clutch: {e}")
        
    def disengage_left_clutch(self):
        """Disengage the left clutch."""
        try:
            GPIO.output(self.left_clutch_pin, GPIO.LOW)
        except Exception as e:
            logger.error(f"Failed to disengage left clutch: {e}")
        
    def engage_right_clutch(self):
        """Engage the right clutch."""
        try:
            GPIO.output(self.right_clutch_pin, GPIO.HIGH)
        except Exception as e:
            logger.error(f"Failed to engage right clutch: {e}")
        
    def disengage_right_clutch(self):
        """Disengage the right clutch."""
        try:
            GPIO.output(self.right_clutch_pin, GPIO.LOW)
        except Exception as e:
            logger.error(f"Failed to disengage right clutch: {e}")
        
    def close(self):
        """Cleanup GPIO pins."""
        if GPIO is not None:
            # Don't call GPIO.cleanup() globally to avoid affecting other users; only cleanup pin
            try:
                GPIO.cleanup(self.left_clutch_pin)
                GPIO.cleanup(self.right_clutch_pin)
            except Exception:
                pass