import time
import math

from structure.commands.Command import Command
from Robot.subsystems.MotorControl import MotorControl
from Robot.subsystems.sensors.IMU import IMU


class DriveFigure8(Command):
    """Drive the vehicle in an open-loop figure‑8 pattern.

    The command does **not** rely on any localization subsystem; all
    steering decisions are based solely on elapsed time.  A simple
    sinusoidal steering profile is used to alternate between left and
    right turns.  The output is clamped by :class:`MotorControl` to the
    allowed range (-30..30 degrees), but the math below already respects
    those limits.

    Once started the command runs until it is interrupted by another
    command or the robot is shut down (``is_finished`` always returns
    ``False``).  The loop period can be adjusted with ``self._period``.
    """

    def __init__(self, motor_control: MotorControl, speed_percent: int = 100, period: float = 10.0):
        super().__init__()
        self.motor_control = motor_control
        self.add_requirement(motor_control)
        self._imu = IMU()

        # runtime parameters
        self.speed_percent = speed_percent            # forward speed
        self._period = float(period)                  # seconds for one full left+right cycle

        # state filled in initialize()
        self._start_time: float | None = None

    def initialize(self):
        # remember when we began; subsequent calls to execute use this to
        # compute elapsed time and steer accordingly.
        self._start_time = time.time()

    def execute(self):
        # compute how long we've been running
        if self._start_time is None:
            # defensive; should not happen
            self._start_time = time.time()
        elapsed = time.time() - self._start_time

        # a very simple steering law that oscillates between -30 and +30
        # degrees with period ``self._period``.  The resulting path is a
        # symmetric left‑right pattern; when run at constant forward speed
        # the vehicle will roughly trace a figure‑eight shape.
        angle_deg = 30.0 * math.sin((2.0 * math.pi / self._period) * elapsed)

        # clamp for safety (motor_control also clamps internally)
        angle_deg = max(-30.0, min(30.0, angle_deg))

        self.motor_control.set_speed_angle(self.speed_percent, int(angle_deg))

    def end(self, interrupted):
        # stop the motors when the command finishes or is preempted
        self.motor_control.set_speed_angle(0, 0)

    def is_finished(self):
        return self._imu.is_calibrated()