#!/usr/bin/env python3

import time
import threading
from datetime import datetime
import logging
from Robot.Constants import Constants
from Robot.subsystems.KalmanStateEstimator import KalmanStateEstimator

logger = logging.getLogger(f"{__name__}.BackWheelEncoder")
logger.setLevel(logging.INFO)  # Set to INFO for detailed output

import RPi.GPIO as GPIO

class BackWheelEncoder:
    
    _instance = None

    # When a new instance is created, sets it to the same global instance
    def __new__(cls):
        # If the instance is None, create a new instance
        # Otherwise, return already created instance
        if cls._instance is None:
            cls._instance = super().__new__(cls)
        return cls._instance
    
    def start(self):
        """Create a proximity sensor reader.

        Args:
            pin: BCM GPIO pin number to read (e.g., 17)
            active_high: True if sensor output is HIGH when object detected
            pull_up: True to enable pull-up, False for pull-down (when using internal pull)
            debounce_ms: Debounce time in milliseconds
            edge: 'rising', 'falling', or 'both' (which edge to detect)
        """
        self.pin_right = Constants.back_right_encoder_pin
        self.pin_left = Constants.back_right_encoder_pin
        self.active_high = True
        self.pull_up = True
        self.debounce_ms = 1
        self.edge = 'rising'

        self.interval = 1  # update interval in seconds
        self._running = False
        self._lock_left = threading.Lock()
        self._lock_right = threading.Lock()
        self._last_event_time = 0.0
        self._count_left = 0
        self._count_right = 0
        self._velocity = 0.0  # m/s
        self._last_update_time = time.time()
        self.state_estimator = KalmanStateEstimator()
        logger.debug("edge {edge}, debounce {debounce}ms".format(edge=self.edge, debounce=self.debounce_ms))
        
        # Wheel parameters
        self.wheel_circumference = Constants.wheel_circumference  # meters (adjust to your wheel)
        self.counts_per_revolution = Constants.counts_per_revolution  # encoder pulses per wheel rotation

        self.run()

    def _gpio_callback_right(self, channel):
        with self._lock_right:
            self._count_right += 1

    def _gpio_callback_left(self, channel):
        with self._lock_left:
            self._count_left += 1

    def run(self):
        """Start monitoring the GPIO pin"""
        # Prevent multiple threads from being started
        if self._running:
            logger.warning(f"Encoder already running on GPIO {self.pin_right} and {self.pin_left}. Ignoring run() call.")
            return
        
        self._running = True

        GPIO.setmode(GPIO.BCM)

        pud = GPIO.PUD_UP if self.pull_up else GPIO.PUD_DOWN
        GPIO.setup(self.pin_right, GPIO.IN, pull_up_down=pud)
        GPIO.setup(self.pin_left, GPIO.IN, pull_up_down=pud)

        # Determine edge type
        if self.edge == 'both':
            gedge = GPIO.BOTH
        elif self.edge == 'rising':
            gedge = GPIO.RISING
        elif self.edge == 'falling':
            gedge = GPIO.FALLING
        else:
            gedge = GPIO.BOTH

        GPIO.add_event_detect(self.pin_right, gedge, callback=self._gpio_callback_right, bouncetime=self.debounce_ms)
        GPIO.add_event_detect(self.pin_left, gedge, callback=self._gpio_callback_left, bouncetime=self.debounce_ms)
        logger.info(f"Started monitoring GPIO {self.pin_right} and {self.pin_left} (active_high={self.active_high})")
        
        def _update_loop():
            while True:
                self.update()
        
        threading.Thread(target=_update_loop, daemon=True).start()

    def update(self):
        """Update velocity estimate from encoder counts"""
        current_time = time.time()
        dt = current_time - self._last_update_time
            
        if dt > 0:
            # Calculate velocity from count changes
            count = self.get_count_total_and_reset() / 2.0  # Average of left and right counts
            distance = (count / self.counts_per_revolution) * self.wheel_circumference
            
            #logger.info(f"count ={self._count} reset for next interval")
            self._velocity = distance / dt
            logger.debug(f"Encoder pin {self.pin_left} velocity: {self._velocity:.3f} m/s over dt={dt:.3f}s with count={count}")
            self.state_estimator.update_encoder_velocity(self._velocity)
                
            # Reset for next interval
            self._last_update_time = current_time
        
        time.sleep(self.interval)

    def close(self):
        """Stop monitoring and cleanup"""
        self._running = False

        if GPIO is not None:
            try:
                GPIO.remove_event_detect(self.pin_left)
                GPIO.remove_event_detect(self.pin_right)
            except Exception:
                pass
            # Don't call GPIO.cleanup() globally to avoid affecting other users; only cleanup pin
            try:
                GPIO.cleanup(self.pin_left)
                GPIO.cleanup(self.pin_right)
            except Exception:
                pass

    def get_count_left(self) -> int:
        """Return the number of callbacks since last reset and reset the counter."""
        with self._lock_left:
            c = self._count_left
        return c

    def get_count_right(self) -> int:
        """Return the number of callbacks since last reset and reset the counter."""
        with self._lock_right:
            c = self._count_right
        return c

    def get_count_total_and_reset(self) -> int:
        """Return the number of callbacks since last reset and reset the counter."""
        left = self.get_count_left()  # Reset left count
        right = self.get_count_right()  # Reset right count
        with self._lock_left, self._lock_right:
            self._count_left = 0
            self._count_right = 0
        return left + right

    def get_velocity(self) -> float:
        """Return current velocity estimate in m/s (forward direction)"""
        return self._velocity

