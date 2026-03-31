import logging
import threading
import time
from typing import Optional
import spidev
from Robot.Constants import Constants

logger = logging.getLogger(f"{__name__}.DriveTrain")
logger.setLevel(logging.DEBUG)  # Set to DEBUG for detailed output

class FrontWheelEncoder:
    NOP = 0x00
    READ_POS = 0x10
    
    def __init__(self):
        try:
            self._spi = spidev.SpiDev()
            self._spi.open(Constants.spi_bus, Constants.frontwheel_encoder_spi_device)
            self._spi.max_speed_hz = Constants.frontwheel_encoder_max_freq_hz
            self._spi.mode = Constants.frontwheel_encoder_spi_mode
            self._resolution = Constants.frontwheel_encoder_resolution
            self._max_position = Constants.frontwheel_encoder_max_position
            self._position = None
            self._running = False
            self._lock = threading.Lock()
            self._interval = 0.02  # 20ms update interval

            logger.info(f"FrontWheelEncoder SPI initialized on bus {Constants.spi_bus}, device {Constants.frontwheel_encoder_spi_device}, mode {Constants.frontwheel_encoder_spi_mode}")
            
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
        
        threading.Thread(target=_update_loop, daemon=True).start()
        
    def get_position(self) -> Optional[float]:
        
        with self._lock:
            if self._position is None:
                return None
            # Convert raw position to angle in degrees
            angle = (self._position / self._max_position) * 360.0
            return angle
    
    def read_position(self) -> Optional[int]:
        """Read raw position data from encoder via SPI.
        
        For SE33SPI encoders, the typical protocol is:
        - Send a read command (or dummy bytes)
        - Receive position data (typically 2 bytes for 12-bit resolution)
        
        Returns:
            Raw position value (0 to max_position) or None on error
        """
        if not self._spi:
            return None

        try:
            # Full transaction in one go
            response = self._spi.xfer2([0xAA, 0xFF, 0xFF, 0xFF])

            # Extract returned bytes
            data_high = response[1]
            data_low  = response[2]
            inv_high  = response[3]
            inv_low   = response[0]  # sometimes wraps depending on timing

            raw = (data_high << 8) | data_low
            inv = (inv_high << 8) | inv_low

            # Validate inverted data
            if (raw ^ 0xFFFF) != inv:
                logger.error("⚠️ SPI data error")
                return None

            # Convert Gray → Binary
            binary = self._gray_to_binary(raw)
            position = self._get_angle(binary)

            with self._lock:
                self._position = position
            
        except Exception as e:
            logger.error(f"FrontWheelEncoder SPI read error: {e}")
            return None
        
    # Gray → Binary conversion
    def _gray_to_binary(self, n):
        result = n
        while n > 0:
            n >>= 1
            result ^= n
        return result
    
    def _get_angle(self, position, bits=14):
        max_val = (1 << bits) - 1
        return (position / max_val) * 360.0
        
    def close(self):
        if self._spi:
            try:
                self._running = False
                self._spi.close()
            except Exception:
                pass