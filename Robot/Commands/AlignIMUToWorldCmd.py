from structure.commands.Command import Command
import time
import math
import numpy as np
from collections import deque
from pathlib import Path
from Robot.Commands.helpers.csvlib import overwrite_csv_with_row, read_last_csv_row
from Robot.subsystems.sensors.IMU import IMU
from Robot.subsystems.sensors.UWB import UWB
import logging

logger = logging.getLogger(f"{__name__}.AlignIMUToWorldCmd")
logger.setLevel(logging.INFO)


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

        # runtime state
        self._start_time: float | None = None
        self._last_time: float | None = None
        self._samples = 0
        self._stable_count = 0
        self._bias = 0.0  # radians
        self.residuals_window = deque(maxlen=100)
        self._last_csv_log_time: float | None = None

        project_root = Path(__file__).resolve().parents[2]
        self._yaw_offset_dir = project_root / "records"
        self._yaw_offset_csv = self._yaw_offset_dir / "yaw_offset.csv"
        self._yaw_offset_fieldnames = ["timestamp", "source", "yaw_offset_deg", "yaw_offset_rad"]

    def _read_last_logged_yaw_offset_deg(self) -> float | None:
        try:
            row = read_last_csv_row(str(self._yaw_offset_csv))
            if row is None:
                return None
            return float(row["yaw_offset_deg"])
        except Exception as exc:
            logger.warning(f"AlignIMUToWorldCmd: failed to read saved yaw offset from CSV: {exc}")
            return None
    
    def _record_yaw_offset_csv(self, timestamp: float, source: str):
        self._yaw_offset_dir.mkdir(parents=True, exist_ok=True)
        row = {
            "timestamp": timestamp,
            "source": source,
            "yaw_offset_deg": math.degrees(self._bias),
            "yaw_offset_rad": self._bias,
        }

        ok = overwrite_csv_with_row(
            filename=str(self._yaw_offset_csv),
            fieldnames=self._yaw_offset_fieldnames,
            row=row,
        )
        if not ok:
            logger.warning("AlignIMUToWorldCmd: failed to write yaw offset CSV row")

    def initialize(self):
        self._start_time = time.time()
        self._last_time = self._start_time
        self._last_csv_log_time = self._start_time
        self._samples = 0
        self._stable_count = 0
        imu = self._imu

        saved_offset_deg = self._read_last_logged_yaw_offset_deg()
        if saved_offset_deg is not None:
            self._bias = math.radians(saved_offset_deg)
            imu.set_yaw_offset(saved_offset_deg)
            logger.info(f"AlignIMUToWorldCmd: restored yaw offset from CSV: {saved_offset_deg:.3f} deg")
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
        
        # Get IMU yaw in radians (IMU.get_angle returns degrees: (yaw, roll, pitch))
        imu = self._imu
        imu_yaw_deg = imu.get_angle()[2]
        imu_yaw_rad = math.radians(imu_yaw_deg)

        logger.debug(f"AlignIMUToWorldCmd: UWB yaw = {math.degrees(uwb_yaw):.3f} deg")
        logger.debug(f"AlignIMUToWorldCmd: IMU yaw = {imu_yaw_deg:.3f} deg (with offset {math.degrees(self._bias):.3f} deg)")

        # residual between measured UWB yaw and corrected IMU yaw
        residual = _wrap_angle(uwb_yaw - imu_yaw_rad)
        self.residuals_window.append(residual)
        self._bias = float(np.median(self.residuals_window))

        # apply bias to IMU (degrees)
        logger.debug(f"AlignIMUToWorldCmd: applying yaw offset {math.degrees(self._bias):.3f} deg")
        imu.set_yaw_offset(math.degrees(self._bias))

        if self._last_csv_log_time is None or (now - self._last_csv_log_time) >= 30.0:
            self._record_yaw_offset_csv(now, "execute")
            self._last_csv_log_time = now

        # bookkeeping
        self._samples += 1
        if abs(residual) < self.tol:
            self._stable_count += 1
        else:
            self._stable_count = 0

        self._last_time = now

    def end(self, interrupted):
        self._record_yaw_offset_csv(time.time(), "end")

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