import logging
import threading
import time
import spidev
from Robot.Constants import Constants
import RPi.GPIO as GPIO

logger = logging.getLogger(f"{__name__}.DriveTrain")
logger.setLevel(logging.INFO)  # Set to DEBUG for detailed output

class DAC:
	def __init__(self,):
		try:
			self.cs_pin = Constants.bitbang_cs_DAC_pin
			self.clk_pin = Constants.bitbang_clock_pin
			self.data_pin = Constants.bitbang_MOSI_pin
			self._max_value = Constants.dac_max_value
			self._setup_delay = Constants.bitbang_setup_delay
			self._clock_delay = Constants.bitbang_clock_delay
			self._available = True
			self._bitbang_spi_lock = Constants.bitbang_spi_lock

			GPIO.setmode(GPIO.BCM)
			GPIO.setwarnings(False)

			GPIO.setup(self.cs_pin, GPIO.OUT)
			GPIO.setup(self.clk_pin, GPIO.OUT)
			GPIO.setup(self.data_pin, GPIO.OUT)

			GPIO.output(self.cs_pin, GPIO.HIGH)
			GPIO.output(self.clk_pin, GPIO.LOW)
			GPIO.output(self.data_pin, GPIO.LOW)

			logger.info(
				"BitBangDAC initialized on CS=%s CLK=%s DATA=%s",
				self.cs_pin,
				self.clk_pin,
				self.data_pin,
			)

		except Exception as e:
			logger.error(f"Failed to initialize BitBangDAC: {e}")
			self._available = False  # Set to False to allow no-op in write

	def write(self, channel: int, input_value: float) -> None:
		"""Write a normalized value to the selected DAC channel.

		Args:
			channel: DAC channel index. Channel 0 and 1 are supported.
			input_value: Normalized output in the range 0.0 .. 1.0.
		"""
		if not self._available:
			return

		value = int(input_value * self._max_value)
		value = max(0, min(self._max_value, value))

		if channel == 0:
			command = 0x3000 | value
		else:
			command = 0xB000 | value

		logger.debug(
			"Writing to DAC channel %s: input=%.3f, value=%s, command=0x%04X",
			channel,
			input_value,
			value,
			command,
		)

		self._send_word(command)

	def _send_word(self, word: int) -> None:
		"""Shift out a 16-bit word MSB-first."""
		with self._bitbang_spi_lock:
			GPIO.output(self.cs_pin, GPIO.LOW)
			time.sleep(self._setup_delay)

			for bit in range(15, -1, -1):
				if word & (1 << bit):
					GPIO.output(self.data_pin, GPIO.HIGH)
				else:
					GPIO.output(self.data_pin, GPIO.LOW)

				time.sleep(self._setup_delay)
				GPIO.output(self.clk_pin, GPIO.HIGH)
				time.sleep(self._clock_delay)
				GPIO.output(self.clk_pin, GPIO.LOW)

			time.sleep(self._setup_delay)
			GPIO.output(self.cs_pin, GPIO.HIGH)

	def close(self) -> None:
		if not self._available:
			return
		try:
			GPIO.output(self.cs_pin, GPIO.HIGH)
			GPIO.output(self.clk_pin, GPIO.LOW)
			GPIO.output(self.data_pin, GPIO.LOW)
			self._available = False
		finally:
			GPIO.cleanup()