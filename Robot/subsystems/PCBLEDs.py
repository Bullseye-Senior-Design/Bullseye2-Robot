import RPi.GPIO as GPIO

from Robot.Constants import Constants
from structure.Subsystem import Subsystem
import logging

logger = logging.getLogger(f"{__name__}.PCBLEDs")
logger.setLevel(logging.INFO)  # Set to INFO for detailed output

class PCBLEDs(Subsystem):
    
    def __init__(self):
        """Initialize PCB LED control GPIO pins and state."""
        # GPIO pin setup code here

        self.status_pin_green = Constants.status_led_green
        self.status_pin_red = Constants.status_led_red
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.status_pin_green, GPIO.OUT)
        GPIO.setup(self.status_pin_red, GPIO.OUT)

    def set_green_status_led(self, isOn):
        if isOn:
            GPIO.output(self.status_pin_green, GPIO.HIGH)
        else:
            GPIO.output(self.status_pin_green, GPIO.LOW)

    def set_red_status_led(self, isOn):
        if isOn:
            GPIO.output(self.status_pin_red, GPIO.HIGH)
        else:
            GPIO.output(self.status_pin_red, GPIO.LOW)

    def close(self):
        """Cleanup GPIO pins."""
        if GPIO is not None:
            # Don't call GPIO.cleanup() globally to avoid affecting other users; only cleanup pin
            try:
                GPIO.cleanup(self.status_pin_green)
                GPIO.cleanup(self.status_pin_red)
            except Exception:
                pass
