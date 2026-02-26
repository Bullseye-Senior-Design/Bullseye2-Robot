import logging
import time
import threading
import RPi.GPIO as GPIO

logger = logging.getLogger("LimitSwitchReader")
logging.basicConfig(level=logging.DEBUG)

class LimitSwitchReader:
    def __init__(self, pin: int, active_high: bool = True, pull_up: bool = True, debounce_ms: int = 50,
                edge: str = 'both'):
        """Create a gpio pin reader.

        Args:
            pin: BCM GPIO pin number to read (e.g., 17)
            active_high: True if sensor output is HIGH when object detected
            pull_up: True to enable pull-up, False for pull-down (when using internal pull)
            debounce_ms: Debounce time in milliseconds
            edge: 'rising', 'falling', or 'both' (which edge to detect)
        """
        self.pin = pin
        self.active_high = active_high
        self.pull_up = pull_up
        self.debounce_ms = debounce_ms
        self.edge = edge.lower()

        self._running = False
        self._lock = threading.Lock()
        self.isTriggered = False

    def _gpio_callback(self, channel):
        # Read the current pin state to determine edge direction
        current_value = GPIO.input(channel)

        # Set state based on edge detection
        # If current value is HIGH, it was a rising edge -> state = True
        # If current value is LOW, it was a falling edge -> state = False
        with self._lock:
            self.isTriggered = not bool(current_value) if self.active_high else bool(current_value)

        logger.debug(f"GPIO callback triggered on pin {channel}, isTriggered={self.isTriggered}, current_value={current_value}")

    def start(self):
        """Start monitoring the GPIO pin"""
        self._running = True

        GPIO.setmode(GPIO.BCM)

        pud = GPIO.PUD_UP if self.pull_up else GPIO.PUD_DOWN
        GPIO.setup(self.pin, GPIO.IN, pull_up_down=pud)

        # Determine edge type
        if self.edge == 'both':
            gedge = GPIO.BOTH
        elif self.edge == 'rising':
            gedge = GPIO.RISING
        elif self.edge == 'falling':
            gedge = GPIO.FALLING
        else:
            gedge = GPIO.BOTH

        GPIO.add_event_detect(self.pin, gedge, callback=self._gpio_callback, bouncetime=self.debounce_ms)
        logger.info(f"Started monitoring GPIO {self.pin} (active_high={self.active_high})")

    def get_isTriggered(self) -> bool:
        """Get the current state of the limit switch."""
        with self._lock:
            return self.isTriggered

    def close(self):
        """Stop monitoring and cleanup"""
        self._running = False

        if GPIO is not None:
            try:
                GPIO.remove_event_detect(self.pin)
            except Exception:
                pass
            # Don't call GPIO.cleanup() globally to avoid affecting other users; only cleanup pin
            try:
                GPIO.cleanup(self.pin)
            except Exception:
                pass