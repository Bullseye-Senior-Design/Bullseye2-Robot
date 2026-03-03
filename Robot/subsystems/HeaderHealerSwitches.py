import time
import threading
from datetime import datetime
import logging

from Robot.Constants import Constants
from Robot.subsystems.subsystemChildren.LimitSwitchReader import LimitSwitchReader
from structure.Subsystem import Subsystem

logger = logging.getLogger("HeaderHealerSwitches")
logging.basicConfig(level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s")

import RPi.GPIO as GPIO

class HeaderHealerSwitches(Subsystem):
	def __init__(self):
		try:
			self.header_switch_pin = Constants.header_limit_switch_pin
			self.healer_switch_pin = Constants.healer_limit_switch_pin
			self.header_switch_reader = LimitSwitchReader(pin=self.header_switch_pin, active_high=True, pull_up=True, debounce_ms=1, edge='both')
			self.healer_switch_reader = LimitSwitchReader(pin=self.healer_switch_pin, active_high=True, pull_up=True, debounce_ms=1, edge='both')
			self.header_switch_reader.start()
			self.healer_switch_reader.start()
		except Exception as e:
			logger.error(f"Failed to initialize HeaderHealerSwitches: {e}")
	
	def get_header_switch_triggered(self) -> bool:
		return self.header_switch_reader.get_isTriggered()

	def get_healer_switch_triggered(self) -> bool:
		return self.healer_switch_reader.get_isTriggered()
	
	def close(self):
		self.header_switch_reader.close()
		self.healer_switch_reader.close()


