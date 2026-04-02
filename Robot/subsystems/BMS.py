import threading
import time
import logging

import serial
from Comms.BatteryData import BatteryData
from Robot.Constants import Constants
from structure.Subsystem import Subsystem

# ==== LOGGING CONFIGURATION ====
logger = logging.getLogger(f"{__name__}.BMS")
logger.setLevel(logging.INFO)  # Set to DEBUG for detailed output

# Fields we care about
TARGET_FIELDS = {"V", "I", "P", "SOC", "TTG"}

class BMS(Subsystem):
    """BMS subsystem for reading battery data from a Victron SmartShunt."""

    def __init__(self):
        super().__init__()
        self._ser = None
        self.battery_data = BatteryData(
            voltage=0.0,
            current=0.0,
            power=0.0,
            state_of_charge=0.0,
            time_remaining=0.0
        )
        self.lock = threading.Lock()
        self.interval = Constants.bms_update_interval

        self._connect_serial()
        
    def _connect_serial(self):
        try:
            self._ser = serial.Serial(Constants.bms_serial_port, Constants.serial_baud_rate, timeout=1)
        except serial.SerialException as e:
            logger.debug(f"[BMS] Could not open serial port {Constants.bms_serial_port}: {e}")
            self._ser = None

    def update(self):
        """Periodically called to update battery data."""
        while True:
            battery_data = self.read_smartshunt()
            if battery_data:
                with self.lock:
                    self.battery_data = battery_data
            time.sleep(self.interval)

    def read_smartshunt(self):
        """Read one SmartShunt packet and return BatteryData."""
        if self._ser is None:
            self._connect_serial()
            return None

        data = {}
        while True:
            line = self._ser.readline().decode(errors="ignore").strip()

            # VE.Direct packets end with a checksum
            if line.startswith("Checksum"):
                if data:
                    return BatteryData(
                        voltage=data.get('V', 0.0),
                        current=data.get('I', 0.0),
                        power=data.get('P', 0.0),
                        state_of_charge=data.get('SOC', 0.0),
                        time_remaining=data.get('TTG', 0.0)
                    )
                data = {}
                continue

            if "\t" in line:
                key, value = line.split("\t", 1)
                if key in TARGET_FIELDS:
                    data[key] = self.parse_value(key, value)

    def get_battery_data(self) -> BatteryData:
        """Get the latest battery data (thread-safe)."""
        with self.lock:
            return self.battery_data

    def close(self):
        """Close the serial connection."""
        if self._ser is not None:
            try:
                self._ser.close()
            except Exception:
                pass
            self._ser = None

    def parse_value(self, key, value):
        """Convert Victron raw values to human-readable units."""
        if key == "V":      # mV -> V
            return float(value) / 1000
        elif key == "I":    # mA -> A
            return float(value) / 1000
        elif key == "P":    # W
            return float(value)
        elif key == "SOC":  # 0.1% -> %
            return float(value) / 10
        elif key == "TTG":  # minutes
            num = float(value)
            return num  # Return as float for time_remaining
        else:
            return value
