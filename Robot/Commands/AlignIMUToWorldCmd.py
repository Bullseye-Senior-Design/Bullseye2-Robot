from structure.commands.Command import Command
import time
import math
import numpy as np
from collections import deque
from helpers.dbConstants import YAW_OFFSET_TABLE
from helpers.sqllib import SQLiteFileManager
from Robot.subsystems.sensors.IMU import IMU
from Robot.subsystems.sensors.UWB import UWB
import logging

logger = logging.getLogger(f"{__name__}.AlignIMUToWorldCmd")
logger.setLevel(logging.DEBUG)


def _wrap_angle(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class AlignIMUToWorldCmd(Command):
    """Command to estimate and apply a yaw offset between IMU and UWB.

    This implements a scalar bias estimator using a sliding-window median that updates
    the IMU yaw offset via `IMU().set_yaw_offset(...)` so the IMU and UWB
    headings align. The estimator runs until `duration` elapses or the
    residual is stable for several samples.
    """
    def __init__(self, imu: IMU, uwb: UWB, tau: float = 5.0, tol_rad: float = 0.01, min_samples: int = 100):
        super().__init__()
        # time constant (seconds) for bias adaptation
        self.tau = tau
        # convergence tolerance on yaw residual (radians)
        self.tol = tol_rad
        # minimum samples before allowing early finish
        self.min_samples = min_samples
        
        self._uwb = uwb
        self._imu = imu
        self._db = SQLiteFileManager()

        # runtime state
        self._start_time: float | None = None
        self._last_time: float | None = None
        self._samples = 0
        self._stable_count = 0
        self._bias = 0.0  # radians
        self.residuals_window = deque(maxlen=100)
        self._last_db_log_time: float | None = None

        self._yaw_offset_table = YAW_OFFSET_TABLE

    def _read_last_logged_yaw_offset_deg(self) -> float | None:
        row = self._db.read_last_row(self._yaw_offset_table)
        if row is None:
            return None
        return float(row["yaw_offset_deg"])
    
    def _record_yaw_offset_db(self, timestamp: float, source: str):
        row = YAW_OFFSET_TABLE.build_row(timestamp, source, math.degrees(self._bias), self._bias)

        ok = self._db.overwrite_with_row(
            table=self._yaw_offset_table,
            row=row,
        )
        if not ok:
            logger.warning("AlignIMUToWorldCmd: failed to write yaw offset row to SQLite")

    def initialize(self):
        self._start_time = time.time()
        self._last_time = self._start_time
        self._last_db_log_time = self._start_time
        self._samples = 0
        self._stable_count = 0
        imu = self._imu

        saved_offset_deg = self._read_last_logged_yaw_offset_deg()
        if saved_offset_deg is not None:
            self._bias = math.radians(saved_offset_deg)
            imu.set_yaw_offset(self._bias)
            logger.info(f"AlignIMUToWorldCmd: restored yaw offset from SQLite: {saved_offset_deg:.3f} deg")
        else:
            self._bias = 0.0
        self.residuals_window.clear()

        logger.info(f"AlignIMUToWorldCmd: starting yaw-bias estimation (tau={self.tau}s, tol={math.degrees(self.tol):.2f} deg, min_samples={self.min_samples})")

    def execute(self):
        now = time.time()

        # Get instantaneous UWB yaw (radians)
        uwb = self._uwb
        uwb_yaw = uwb.get_angle()
        if uwb_yaw is None:
            #  logger.warning("AlignIMUToWorldCmd: insufficient UWB tags to compute heading; skipping this cycle")
            self._start_time = time.time()  # reset start time to avoid premature timeout
            return
        
        # Get IMU yaw in radians (IMU.get_angle returns (roll, pitch, yaw) in radians)
        imu = self._imu
        imu_yaw_rad = float(imu.get_angle()[2])
        imu_raw_yaw_rad = float(imu.get_raw_angle()[2])
        imu_offset_deg = math.degrees(imu.yaw_offset_rad)

        logger.debug(f"AlignIMUToWorldCmd: UWB yaw = {math.degrees(uwb_yaw):.3f} deg")
        logger.debug(f"AlignIMUToWorldCmd: IMU raw yaw = {math.degrees(imu_raw_yaw_rad):.3f} deg (before offset)")
        logger.debug(f"AlignIMUToWorldCmd: IMU yaw = {math.degrees(imu_yaw_rad):.3f} deg (with offset {imu_offset_deg:.3f} deg)")

        # residual between measured UWB yaw and corrected IMU yaw
        residual = _wrap_angle(uwb_yaw - imu_yaw_rad)
        self.residuals_window.append(residual)
        self._bias = float(np.median(self.residuals_window))


        if self._last_db_log_time is None or (now - self._last_db_log_time) >= 30.0:
            # apply bias to IMU (degrees)
            logger.debug(f"AlignIMUToWorldCmd: applying yaw offset {math.degrees(self._bias):.3f} deg")
            imu.set_yaw_offset(self._bias)
            self._record_yaw_offset_db(now, "execute")
            self._last_db_log_time = now

        # bookkeeping
        self._samples += 1
        if abs(residual) < self.tol:
            self._stable_count += 1
        else:
            self._stable_count = 0

        self._last_time = now

    def end(self, interrupted):
        self._record_yaw_offset_db(time.time(), "end")

        if interrupted:
            logger.info("AlignIMUToWorldCmd interrupted; leaving current IMU yaw offset in place")
        else:
            logger.info(f"AlignIMUToWorldCmd completed: applied yaw offset {math.degrees(self._bias):.3f} deg")

    def is_finished(self) -> bool:
        # shouldn't happen, but guard against None
        if self._start_time is None:
            return True
        
        if self._samples >= self.min_samples and self._stable_count >= 10:
            return True
        return False