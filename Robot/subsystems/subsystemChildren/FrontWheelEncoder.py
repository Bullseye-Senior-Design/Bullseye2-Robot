import logging
import math
import threading
import time
from typing import Optional
import spidev
from Robot.Constants import Constants
import RPi.GPIO as GPIO
from Robot.MathUtil import MathUtil
import math


logger = logging.getLogger(f"{__name__}.DriveTrain")
logger.setLevel(logging.INFO)  # Set to DEBUG for detailed output

class FrontWheelEncoder:
    def __init__(self):
        try:
            self.cs_pin = Constants.bitbang_cs_frontwheel_encoder_pin
            self.clock_pin = Constants.bitbang_clock_pin
            self.data_pin = Constants.bitbang_MISO_pin
            self.shifter_dir_pin = Constants.shifter_dir_pin
            self._max_position = Constants.frontwheel_encoder_max_position
            self._bitbang_spi_lock = Constants.bitbang_spi_lock
            self._bitbang_setup_delay = Constants.bitbang_setup_delay
            self._position = None
            self._running = False
            self._lock = threading.Lock()
            self._interval = 0.1  # 20ms update interval
            
            self.frontwheel_zero_offset = Constants.frontwheel_encoder_zero_offset

            GPIO.setmode(GPIO.BCM)
            GPIO.setup(self.cs_pin, GPIO.OUT)
            GPIO.setup(self.clock_pin, GPIO.OUT)
            # Shifter: LOW is input to Pi
            # Shifter: HIGH is output from Pi
            GPIO.setup(self.shifter_dir_pin, GPIO.OUT)

            # initialize spi lines to known state
            GPIO.output(self.cs_pin,  GPIO.HIGH)
            GPIO.output(self.clock_pin, GPIO.LOW)
            GPIO.output(self.shifter_dir_pin, GPIO.LOW)
            
            self.run()
        except Exception as e:
            logger.error(f"Failed to initialize SPI for FrontWheelEncoder: {e}")
            self._spi = None  # Set to None to allow no-op in _read_raw_position

    def run(self):
        """Start monitoring the GPIO pin"""
        # Prevent multiple threads from being started
        if self._running:
            logger.warning(f"FrontWheel encoder already running. Ignoring run() call.")
            return
        
        self._running = True
        
        def _update_loop():
            while self._running:
                time.sleep(self._interval)
                self.read_position()
        
        self._thread = threading.Thread(target=_update_loop, daemon=True)
        self._thread.start()
        
    def get_position(self) -> Optional[float]:
        """Returns front wheel angle

        Returns:
            Optional[float]: wheel angle in radians, or None if not available
        """
        
        with self._lock:
            angle = self._position
            return angle
    
    def read_position(self):
        try: 
            """Read full frame and return 14-bit angle or -1 on error"""
            with self._bitbang_spi_lock:
                rx = [0] * 10

                GPIO.output(self.cs_pin, GPIO.LOW)
                time.sleep(self._bitbang_setup_delay)

                rx[0] = self.spi_byte(0xAA)
                for i in range(1, 10):
                    rx[i] = self.spi_byte(0xFF)

                GPIO.output(self.cs_pin, GPIO.HIGH)

                # Data bytes: rx[2]=MSB, rx[3]=LSB, rx[4]=~MSB, rx[5]=~LSB
                data = (rx[2] << 8) | rx[3]
                inv  = (rx[4] << 8) | rx[5]

            # Print raw bytes for debugging
            logger.debug("Raw RX: %s", " ".join(f"{b:02X}" for b in rx))

            logger.debug(f"Data   : 0x{data:04X}   ~Data : 0x{inv:04X}   XOR = 0x{data ^ inv:04X}")

            if (data ^ inv) == 0xFFFF:
                raw_angle = ((data & 0x3FFF) * 2.0 * math.pi) / self._max_position
                # Apply calibration offset and keep angle wrapped to [-pi, pi])
                angle = MathUtil.wrap_to_pi(raw_angle - self.frontwheel_zero_offset)
                logger.debug(f"→ VALID Degrees: {math.degrees(angle):.2f}")
                with self._lock:
                    self._position = angle
            else:
                logger.debug("→ CRC / Communication Error")
        except Exception as e:
            logger.error(f"Error reading front wheel encoder: {e}")
    
    def spi_byte(self, tx):
        """Send one byte and read response (open-drain style on DAT)"""
        rx = 0
        for bit in range(7, -1, -1):
            # Drive bit (1 = high-Z so pull-up on 5V side wins, 0 = drive low)
            if tx & (1 << bit):
                GPIO.output(self.shifter_dir_pin, GPIO.LOW) # input to Pi
                GPIO.setup(self.data_pin, GPIO.IN)          # high-Z
            else:
                GPIO.output(self.shifter_dir_pin, GPIO.HIGH) # output from Pi
                GPIO.setup(self.data_pin, GPIO.OUT)
                GPIO.output(self.data_pin, GPIO.LOW)         # pull MISO low

            GPIO.output(self.clock_pin, GPIO.HIGH)            # clock rising edge
            if GPIO.input(self.data_pin):
                rx |= (1 << bit)
            GPIO.output(self.clock_pin, GPIO.LOW)             # falling edge
        return rx
        
    def close(self):
        """Stop the update thread and clean up GPIO"""
        self._running = False
        if hasattr(self, "_thread"):
            self._thread.join(timeout=1)
        GPIO.cleanup([self.cs_pin, self.clock_pin, self.data_pin, self.shifter_dir_pin])