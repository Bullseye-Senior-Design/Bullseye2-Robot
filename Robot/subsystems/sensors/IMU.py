import logging
import math

import serial
import time
import struct
import threading
from typing import Optional, Tuple, Dict
import numpy as np
from Robot.Constants import Constants
from Robot.MathUtil import MathUtil
from Robot.subsystems.algorithms.KalmanStateEstimator import KalmanStateEstimator

logger = logging.getLogger(f"{__name__}.IMU")
logger.setLevel(logging.DEBUG)  # Set to INFO for high-level events, DEBUG for detailed parsing info

# Using Hiwonder IMU (WitMotion WT901) as the primary IMU. Communicates over serial with a custom binary protocol.
class IMU:
    _instance = None

    # When a new instance is created, sets it to the same global instance
    def __new__(cls):
        # If the instance is None, create a new instance
        # Otherwise, return already created instance
        if cls._instance is None:
            cls._instance = super().__new__(cls)
            cls._instance._start()
        return cls._instance

    def _start(self):
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
        self.angle: Tuple[float, float, float] = (0.0, 0.0, 0.0)   # Corrected Roll, Pitch, Yaw in rad
        self.raw_angle: Tuple[float, float, float] = (0.0, 0.0, 0.0)   # Raw Roll, Pitch, Yaw in rad
        self.mag: Tuple[float, float, float] = (0.0, 0.0, 0.0)     # Magnetometer raw units
        self.quaternion: Tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)  # Corrected qx, qy, qz, qw
        self.raw_quaternion: Tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)  # Raw qx, qy, qz, qw
        self.yaw_offset_rad: float = 0.0

        self.state_estimator = KalmanStateEstimator()

        self.buffer = bytearray()
        self.last_update_time = 0.0

        if self.connect():
            self.start()

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

    def _parse_packet(self, packet: bytes) -> bool:
        """Parse one 11-byte WitMotion packet. Returns True if valid, False if invalid."""
        # FIX Fault 2: Function now returns a boolean so the reader loop knows if it should drop 1 byte or 11.
        if len(packet) != 11 or packet[0] != 0x55:
            return False

        pkt_type = packet[1]
        data = packet[2:10]
        checksum = sum(packet[0:10]) & 0xFF
        if checksum != packet[10]:
            return False  # Invalid checksum

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

            elif pkt_type == 0x54:  # Magnetometer
                mx = float(struct.unpack('<h', data[0:2])[0])
                my = float(struct.unpack('<h', data[2:4])[0])
                mz = float(struct.unpack('<h', data[4:6])[0])
                with self._lock:
                    self.mag = (mx, my, mz)
                    self.last_update_time = time.time()

            elif pkt_type == 0x59:  # Quaternion
                qw = struct.unpack('<h', data[0:2])[0] / 32768.0
                qx = struct.unpack('<h', data[2:4])[0] / 32768.0
                qy = struct.unpack('<h', data[4:6])[0] / 32768.0
                qz = struct.unpack('<h', data[6:8])[0] / 32768.0
                
                # FIX Fault 3: Normalize the quaternion to prevent rotation "stretching" or Kalman filter instability
                mag = math.sqrt(qw*qw + qx*qx + qy*qy + qz*qz)
                if mag > 0:
                    qw, qx, qy, qz = qw/mag, qx/mag, qy/mag, qz/mag
                else:
                    qw, qx, qy, qz = 1.0, 0.0, 0.0, 0.0  # Fallback to identity

                with self._lock:
                    self.raw_quaternion = (qx, qy, qz, qw)
                    self.raw_angle = self._quat_to_euler(self.raw_quaternion)
                    self.angle = self._apply_yaw_offset_to_euler(self.raw_angle)
                    self.quaternion = self._apply_yaw_offset_to_quaternion(self.raw_quaternion)
                    
                    self.last_update_time = time.time()

            return True

        except Exception:
            return False  # Ignore bad packets and let the loop re-align

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
                        
                        # FIX Fault 2: Only delete 11 bytes if the packet is actually valid.
                        # Otherwise, delete 1 byte so we can search for the next valid 0x55 header.
                        is_valid = self._parse_packet(pkt)
                        if is_valid:
                            del self.buffer[:11]
                        else:
                            del self.buffer[0]
                    else:
                        del self.buffer[0]  # Remove garbage

                self.state_estimator.update_imu_attitude(q_meas=np.array(self.get_quaternion()))

                time.sleep(1.0 / (self.update_rate * 2))  # Low CPU usage

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
                'raw_angle': self.raw_angle,
                'mag': self.mag,
                'quaternion': self.quaternion,
                'raw_quaternion': self.raw_quaternion,
                'timestamp': self.last_update_time
            }
        
    def set_yaw_offset(self, offset_rad: float):
        """Set a yaw offset (in radians) to align IMU yaw with world frame."""
        with self._lock:
            self.yaw_offset_rad = float(offset_rad)

    def _apply_yaw_offset_to_euler(self, euler_rad: Tuple[float, float, float]) -> Tuple[float, float, float]:
        roll_rad, pitch_rad, yaw_rad = euler_rad
        corrected_yaw = MathUtil.wrap_to_pi(yaw_rad + self.yaw_offset_rad)
        return (float(roll_rad), float(pitch_rad), float(corrected_yaw))

    def _quat_to_euler(self, quat: Tuple[float, float, float, float]) -> Tuple[float, float, float]:
        q = np.asarray(quat, dtype=float)
        if not np.all(np.isfinite(q)):
            return (0.0, 0.0, 0.0)
        euler = MathUtil.quat_to_euler(q)
        return (float(euler[0]), float(euler[1]), float(euler[2]))

    def _quaternion_multiply(self, q1: Tuple[float, float, float, float], q2: Tuple[float, float, float, float]) -> Tuple[float, float, float, float]:
        """Multiplies two quaternions in (x, y, z, w) order."""
        x1, y1, z1, w1 = q1
        x2, y2, z2, w2 = q2
        
        rx = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
        ry = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
        rz = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
        rw = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
        return (rx, ry, rz, rw)

    def _apply_yaw_offset_to_quaternion(self, quat: Tuple[float, float, float, float]) -> Tuple[float, float, float, float]:
        """
        FIX Fault 1: Applies yaw offset purely in quaternion space to prevent wrap-around 
        jumps and Gimbal Lock when converting to and from Euler angles.
        """
        half_yaw = self.yaw_offset_rad / 2.0
        
        # Create a quaternion representing ONLY the yaw offset (rotation around Z axis)
        # format: (x, y, z, w)
        q_offset = (0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw))
        
        # Multiply the offset quaternion by the raw quaternion (q_offset * q_raw)
        return self._quaternion_multiply(q_offset, quat)
        
    def send_command(self, cmd: bytes):
        """Send raw configuration command to the IMU."""
        if self.ser and self.ser.is_open:
            self.ser.write(cmd)
            self.ser.flush()
            # FIX: Increased from 0.05 to 0.1 because WT901 internal EEPROM requires 100ms
            time.sleep(0.1)  

    def set_output_rate(self, rate_hz: int = 100, save: bool = True):
        """
        Set output rate.
        rate_hz: 100 for 100Hz (common values: 10, 20, 50, 100, 200)
        save: True = make persistent (requires re-power after)
        """
        rate_map = {
            0.1: 0x01, 0.5: 0x02, 1: 0x03, 2: 0x04, 5: 0x05,
            10: 0x06, 20: 0x07, 50: 0x08, 100: 0x09,
            125: 0x0A, 200: 0x0B
        }
        
        rate_byte = rate_map.get(rate_hz)
        if rate_byte is None:
            logger.error(f"Unsupported rate {rate_hz} Hz. Use 10, 20, 50, 100, or 200.")
            return False

        # Unlock
        self.send_command(b'\xFF\xAA\x69\x88\xB5')

        # Set rate
        cmd = bytes([0xFF, 0xAA, 0x03, rate_byte, 0x00])
        self.send_command(cmd)

        if save:
            # FIX: WT901 re-locks after every command. You must unlock again to save.
            self.send_command(b'\xFF\xAA\x69\x88\xB5')
            # Save to flash
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
        """Returns corrected (roll, pitch, yaw) in radians."""
        with self._lock:
            return self.angle

    def get_mag(self) -> Tuple[float, float, float]:
        """Returns magnetometer tuple (mx, my, mz) in raw IMU units."""
        with self._lock:
            return self.mag

    def get_quaternion(self) -> Tuple[float, float, float, float]:
        """Returns corrected quaternion as (qx, qy, qz, qw)."""
        with self._lock:
            return self.quaternion

    def get_raw_quaternion(self) -> Tuple[float, float, float, float]:
        """Returns raw (uncorrected) quaternion as (qx, qy, qz, qw)."""
        with self._lock:
            return self.raw_quaternion

    
    def get_raw_angle(self) -> Tuple[float, float, float]:
        """Returns raw (uncorrected) (roll, pitch, yaw) in radians."""
        with self._lock:
            return self.raw_angle

    # FIX: request_calibration_status() was completely deleted. 
    # Sending 0xFF 0xAA 0x01 0x01 0x01 forces the WT901 to set a new zero-bias for Gyro/Acc. 
    # If done while the robot was moving, it destroys all quaternion accuracy.

    def is_alive(self) -> bool:
        """Check if data is being received recently (within last 2 seconds)"""
        with self._lock:
            return (time.time() - self.last_update_time) < 2.0