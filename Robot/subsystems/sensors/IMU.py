import logging

import serial
import time
import struct
import threading
from typing import Optional, Tuple, Dict
from Robot.Constants import Constants
from Robot.subsystems.algorithms.KalmanStateEstimator import KalmanStateEstimator

logger = logging.getLogger(f"{__name__}.IMU")
logger.setLevel(logging.INFO)  # Set to INFO for high-level events, DEBUG for detailed parsing info

class IMU:
    def __init__(self):
        """
        Initialize the IMU.
        """
        self.port = Constants.imu_serial_port
        self.baudrate = Constants.imu_baud_rate
        self.timeout = Constants.imu_timeout
        self.update_rate = Constants.imu_update_rate

        self.ser: Optional[serial.Serial] = None
        self.running = False
        self.thread: Optional[threading.Thread] = None

        # Latest sensor data (thread-safe with lock)
        self._lock = threading.Lock()
        self.acc: Tuple[float, float, float] = (0.0, 0.0, 0.0)      # g
        self.gyro: Tuple[float, float, float] = (0.0, 0.0, 0.0)    # °/s
        self.angle: Tuple[float, float, float] = (0.0, 0.0, 0.0)   # Roll, Pitch, Yaw in °
        self.quaternion: Tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)  # qx, qy, qz, qw
        self.calibration_status: Dict[str, int] = {
            'acc_x': 0,   # 0-3, higher = better calibration
            'acc_y': 0,
            'acc_z': 0,
            'gyro_x': 0,
            'gyro_y': 0,
            'gyro_z': 0
        }

        self.state_estimator = KalmanStateEstimator()

        self.set_output_rate(self.update_rate, save=False)  # Set update rate without saving to flash

        self.buffer = bytearray()
        self.last_update_time = 0.0

    def connect(self) -> bool:
        """Open the serial port."""
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
            logger.info(f"IMU connected on {self.port} at {self.baudrate} baud")
            return True
        except Exception as e:
            logger.error(f"Failed to open {self.port}: {e}")
            logger.info("Check with: ls /dev/tty* and make sure no other program is using the port.")
            return False

    def start(self):
        """Start background reading thread."""
        if self.running:
            return
        if not self.ser or not self.ser.is_open:
            if not self.connect():
                return

        self.running = True
        self.thread = threading.Thread(target=self._reader_loop, daemon=True)
        self.thread.start()

    def close(self):
        """Stop the reading thread and close port."""
        self.running = False
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=1.0)
        if self.ser and self.ser.is_open:
            self.ser.close()
        logger.info("IMU stopped and port closed.")

    def _parse_packet(self, packet: bytes):
        """Parse one 11-byte WitMotion packet."""
        if len(packet) != 11 or packet[0] != 0x55:
            return

        pkt_type = packet[1]
        data = packet[2:10]
        checksum = sum(packet[0:10]) & 0xFF
        if checksum != packet[10]:
            return  # Invalid checksum

        try:
            if pkt_type == 0x51:  # Acceleration (±16g)
                ax = struct.unpack('<h', data[0:2])[0] / 32768.0 * 16.0
                ay = struct.unpack('<h', data[2:4])[0] / 32768.0 * 16.0
                az = struct.unpack('<h', data[4:6])[0] / 32768.0 * 16.0
                with self._lock:
                    self.acc = (ax, ay, az)
                    self.last_update_time = time.time()

            elif pkt_type == 0x52:  # Gyroscope (±2000°/s)
                gx = struct.unpack('<h', data[0:2])[0] / 32768.0 * 2000.0
                gy = struct.unpack('<h', data[2:4])[0] / 32768.0 * 2000.0
                gz = struct.unpack('<h', data[4:6])[0] / 32768.0 * 2000.0
                with self._lock:
                    self.gyro = (gx, gy, gz)
                    self.last_update_time = time.time()

            elif pkt_type == 0x53:  # Euler angles (Roll, Pitch, Yaw)
                roll  = struct.unpack('<h', data[0:2])[0] / 32768.0 * 180.0
                pitch = struct.unpack('<h', data[2:4])[0] / 32768.0 * 180.0
                yaw   = struct.unpack('<h', data[4:6])[0] / 32768.0 * 180.0
                with self._lock:
                    self.angle = (roll, pitch, yaw)
                    self.last_update_time = time.time()

            elif pkt_type == 0x59:  # Quaternion
                # WitMotion sends q0, q1, q2, q3 scaled by 32768.
                # q0 is scalar (w). Convert to estimator order [qx, qy, qz, qw].
                qw = struct.unpack('<h', data[0:2])[0] / 32768.0
                qx = struct.unpack('<h', data[2:4])[0] / 32768.0
                qy = struct.unpack('<h', data[4:6])[0] / 32768.0
                qz = struct.unpack('<h', data[6:8])[0] / 32768.0
                with self._lock:
                    self.quaternion = (qx, qy, qz, qw)
                    self.last_update_time = time.time()

            elif pkt_type == 0x5A:  # Calibration status
                # Bytes 0-1: Reserved/Status
                # Byte 2: Acc calib (bits 0-1: X, bits 2-3: Y, bits 4-5: Z)
                # Byte 3: Gyro calib (bits 0-1: X, bits 2-3: Y, bits 4-5: Z)
                acc_calib_byte = data[2]
                gyro_calib_byte = data[3]
                
                with self._lock:
                    self.calibration_status = {
                        'acc_x': (acc_calib_byte >> 0) & 0x3,
                        'acc_y': (acc_calib_byte >> 2) & 0x3,
                        'acc_z': (acc_calib_byte >> 4) & 0x3,
                        'gyro_x': (gyro_calib_byte >> 0) & 0x3,
                        'gyro_y': (gyro_calib_byte >> 2) & 0x3,
                        'gyro_z': (gyro_calib_byte >> 4) & 0x3
                    }
                    self.last_update_time = time.time()

            # Add more types here later (0x54 Mag, 0x59 Quaternion, 0x56 Baro, etc.)

        except Exception:
            pass  # Ignore bad packets

    def _reader_loop(self):
        """Background thread that continuously reads and parses data."""
        while self.running and self.ser and self.ser.is_open:
            try:
                if self.ser.in_waiting > 0:
                    raw = self.ser.read(self.ser.in_waiting)
                    self.buffer.extend(raw)

                # Process any complete 11-byte packets
                while len(self.buffer) >= 11:
                    if self.buffer[0] == 0x55:
                        pkt = self.buffer[:11]
                        self._parse_packet(pkt)
                        del self.buffer[:11]
                    else:
                        del self.buffer[0]  # Remove garbage

                self.state_estimator.update_imu_attitude(q_meas=self.get_quaternion())

                time.sleep(1.0 / self.update_rate)  # Low CPU usage

            except Exception as e:
                if self.running:
                    logger.error(f"IMU read error: {e}")
                break

    def get_data(self) -> Dict:
        """
        Return the latest sensor data as a dictionary.
        Safe to call from the main thread.
        """
        with self._lock:
            return {
                'acc': self.acc,
                'gyro': self.gyro,
                'angle': self.angle,   # Roll, Pitch, Yaw
                'quaternion': self.quaternion,
                'calibration': self.calibration_status,
                'timestamp': self.last_update_time
            }
        
    def send_command(self, cmd: bytes):
        """Send raw configuration command to the IMU."""
        if self.ser and self.ser.is_open:
            self.ser.write(cmd)
            self.ser.flush()
            time.sleep(0.05)  # Small delay for module to process
            logger.debug(f"Sent command: {' '.join(f'{b:02X}' for b in cmd)}")

    def set_output_rate(self, rate_hz: int = 100, save: bool = True):
        """
        Set output rate.
        rate_hz: 100 for 100Hz (common values: 10, 20, 50, 100, 200)
        save: True = make persistent (requires re-power after)
        """
        # Map Hz to rate byte (from Wit protocol)
        rate_map = {
            0.1: 0x01, 0.5: 0x02, 1: 0x03, 2: 0x04, 5: 0x05,
            10: 0x06, 20: 0x07, 50: 0x08, 100: 0x09,
            125: 0x0A, 200: 0x0B
        }
        
        rate_byte = rate_map.get(rate_hz)
        if rate_byte is None:
            logger.error(f"Unsupported rate {rate_hz} Hz. Use 10, 20, 50, 100, or 200.")
            return False

        # Unlock (often required before config)
        self.send_command(b'\xFF\xAA\x69\x88\xB5')

        # Set rate: 0xFF 0xAA 0x03 RATE 0x00
        cmd = bytes([0xFF, 0xAA, 0x03, rate_byte, 0x00])
        self.send_command(cmd)

        if save:
            # Save to flash: 0xFF 0xAA 0x00 0x00 0x00
            self.send_command(b'\xFF\xAA\x00\x00\x00')
            logger.debug(f"Output rate set to {rate_hz} Hz and saved. Re-power the IMU for changes to take effect.")
        else:
            logger.debug(f"Output rate set to {rate_hz} Hz (temporary).")

        return True

    def get_acc(self) -> Tuple[float, float, float]:
        with self._lock:
            return self.acc

    def get_gyro(self) -> Tuple[float, float, float]:
        with self._lock:
            return self.gyro

    def get_angle(self) -> Tuple[float, float, float]:
        """Returns (roll, pitch, yaw) in degrees"""
        with self._lock:
            return self.angle

    def get_quaternion(self) -> Tuple[float, float, float, float]:
        """Returns quaternion as (qx, qy, qz, qw)."""
        with self._lock:
            return self.quaternion

    def get_calibration_status(self) -> Dict[str, int]:
        """
        Return the calibration status for each axis.
        Values: 0-3 (0 = not calibrated, 3 = fully calibrated)
        Returns: {
            'acc_x': 0-3, 'acc_y': 0-3, 'acc_z': 0-3,
            'gyro_x': 0-3, 'gyro_y': 0-3, 'gyro_z': 0-3
        }
        """
        with self._lock:
            return self.calibration_status.copy()

    def request_calibration_status(self):
        """
        Request calibration status from the IMU.
        Response is asynchronously parsed and stored in self.calibration_status.
        """
        # Command to read calibration status: 0xFF 0xAA 0x01 0x01 0x01
        self.send_command(b'\xFF\xAA\x01\x01\x01')

    def is_alive(self) -> bool:
        """Check if data is being received recently (within last 2 seconds)"""
        with self._lock:
            return (time.time() - self.last_update_time) < 2.0