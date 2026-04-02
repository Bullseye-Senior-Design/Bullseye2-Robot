import logging
import math
import threading
import time
from typing import Optional
import spidev
from Robot.Constants import Constants
import RPi.GPIO as GPIO

logger = logging.getLogger(f"{__name__}.DriveTrain")
logger.setLevel(logging.DEBUG)  # Set to DEBUG for detailed output

class FrontWheelEncoder:
    def __init__(self):
        try:
            self.CS_PIN = Constants.bitbang_cs_frontwheel_encoder_pin
            self.CLK_PIN = Constants.bitbang_clock_pin
            self.DATA_PIN = Constants.bitbang_MISO_pin
            self._max_position = Constants.frontwheel_encoder_max_position
            self._bitbang_spi_lock = Constants.bitbang_spi_lock
            self._position = None
            self._running = False
            self._lock = threading.Lock()
            self._interval = 0.1  # 20ms update interval

            GPIO.setmode(GPIO.BCM)
            GPIO.setup(self.CS_PIN, GPIO.OUT)
            GPIO.setup(self.CLK_PIN, GPIO.OUT)
            GPIO.output(self.CS_PIN,  GPIO.HIGH)
            GPIO.output(self.CLK_PIN, GPIO.LOW)
            
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
            while True:
                time.sleep(self._interval)
                self.read_position()
        
        self._thread = threading.Thread(target=_update_loop, daemon=True)
        self._thread.start()
        
    def get_position(self) -> Optional[float]:
        """Returns front wheel angle

        Returns:
            Optional[float]: _description_
        """
        
        with self._lock:
            if self._position is None:
                return None
            # Convert raw position to angle in degrees
            angle = (self._position / self._max_position) * 360.0
            return angle
    
    def read_position(self):
        """Read full frame and return 14-bit angle or -1 on error"""
        with self._bitbang_spi_lock:
            rx = [0] * 10

            GPIO.output(self.CS_PIN, GPIO.LOW)
            time.sleep(0.01)          # 10 µs minimum after /SS low

            rx[0] = self.spi_byte(0xAA)
            for i in range(1, 10):
                rx[i] = self.spi_byte(0xFF)

            GPIO.output(self.CS_PIN, GPIO.HIGH)

            # Data bytes: rx[2]=MSB, rx[3]=LSB, rx[4]=~MSB, rx[5]=~LSB
            data = (rx[2] << 8) | rx[3]
            inv  = (rx[4] << 8) | rx[5]

        # Print raw bytes for debugging
        logger.debug("Raw RX: %s", " ".join(f"{b:02X}" for b in rx))

        logger.debug(f"Data   : 0x{data:04X}   ~Data : 0x{inv:04X}   XOR = 0x{data ^ inv:04X}")

        if (data ^ inv) == 0xFFFF:
            angle = ((data & 0x3FFF) * 2.0 * math.pi) / self._max_position
            logger.debug(f"→ VALID Radians: {angle:.2f}")
            with self._lock:
                self._position = angle
        else:
            logger.debug("→ CRC / Communication Error")
    
    def spi_byte(self, tx):
        """Send one byte and read response (open-drain style on DAT)"""
        rx = 0
        for bit in range(7, -1, -1):
            # Drive bit (1 = high-Z so pull-up on 5V side wins, 0 = drive low)
            if tx & (1 << bit):
                GPIO.setup(self.DATA_PIN, GPIO.IN)          # high-Z
            else:
                GPIO.setup(self.DATA_PIN, GPIO.OUT)
                GPIO.output(self.DATA_PIN, GPIO.LOW)

            GPIO.output(self.CLK_PIN, GPIO.HIGH)            # clock rising edge
            if GPIO.input(self.DATA_PIN):
                rx |= (1 << bit)
            GPIO.output(self.CLK_PIN, GPIO.LOW)             # falling edge
        return rx
        
    def close(self):
        """Stop the update thread and clean up GPIO"""
        self._running = False
        if hasattr(self, "_thread"):
            self._thread.join(timeout=1)
        GPIO.cleanup([self.CS_PIN, self.CLK_PIN, self.DATA_PIN])