from Robot.Commands.TestingFollowPathCmd import TestingFollowPathCmd
from structure.commands.Command import Command
from Robot.subsystems.sensors.IMU import IMU
import numpy as np
from Robot.MathUtil import MathUtil
from Robot.Commands.FollowPathCmd import FollowPathCmd
from Robot.subsystems.DriveTrain import DriveTrain

from Robot.subsystems.algorithms.PathFollowing import PathFollowing


class ZeroIMUCmd(Command):
    """Command that sets the IMU yaw offset so the current Kalman-relevant heading becomes zero.

    Behavior:
        - Collect up to `sample_count` raw quaternion yaw samples (radians) and
            compute a circular-safe median by unwrapping.
        - Apply yaw offset = -median_heading (radians) using
      `IMU.set_yaw_offset()` and finish.
    """

    def __init__(self,
                 drive_train: DriveTrain,
                 path_following: PathFollowing,
                 schedule_followup: bool = True, 
                 sample_count: int = 10):
        super().__init__()
        self._imu = IMU()
        self.drive_train = drive_train
        self.path_following = path_following
        self.schedule_followup = schedule_followup
        
        self._applied = False
        self.sample_count = int(sample_count)
        self._samples = []

    def initialize(self):
        self._applied = False
        self._samples = []

    def execute(self):
        if self._applied:
            return

        raw_quat = self._imu.get_raw_quaternion()
        if not raw_quat or len(raw_quat) < 4:
            return

        heading_rad = float(MathUtil.quat_to_euler(np.asarray(raw_quat, dtype=float))[2])

        # store sample
        self._samples.append(heading_rad)
        # keep at most sample_count (older values are fine but we only need N)
        if len(self._samples) < self.sample_count:
            # not enough samples yet
            return

        # Compute circular-safe median in radians.
        arr = np.asarray(self._samples, dtype=float)
        unwrapped = np.unwrap(arr)
        median_rad = float(np.median(unwrapped))
        median_rad = ((median_rad + np.pi) % (2.0 * np.pi)) - np.pi

        yaw_offset_rad = -median_rad
        self._imu.set_yaw_offset(yaw_offset_rad)
        self._applied = True
        print(
            "ZeroIMUCmd: applied yaw offset "
            f"{np.degrees(yaw_offset_rad):.2f} deg to zero median heading "
            f"(median was {np.degrees(median_rad):.2f} deg)"
        )

    def end(self, interrupted):
        if interrupted and not self._applied:
            print("ZeroIMUCmd: interrupted before applying yaw offset")
        if self.schedule_followup:
            TestingFollowPathCmd(self.drive_train, self.path_following).schedule()

    def is_finished(self):
        return self._applied