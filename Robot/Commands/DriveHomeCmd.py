from structure.commands.Command import Command
from Robot.subsystems.DriveTrain import DriveTrain
from Robot.subsystems.algorithms.PathFollowing import PathFollowing, DriveDirection
from Robot.subsystems.algorithms.KalmanStateEstimator import KalmanStateEstimator
from Robot.Constants import Constants
import logging

from helpers.dbConstants import HOME_POSITION_TABLE
from helpers.sqllib import SQLiteFileManager


logger = logging.getLogger(f"{__name__}.DriveHomeCmd")

class DriveHomeCmd(Command):
    def __init__(self, drive_train : DriveTrain, path_following : PathFollowing):
        super().__init__()
        self._drive_train = drive_train
        self._path_following = path_following
        self._kalman_estimator = KalmanStateEstimator()
        self._db = SQLiteFileManager()
        self._home_position_key = HOME_POSITION_TABLE
        self.add_requirement(drive_train)
        self.add_requirement(path_following)

    def _read_home_position(self) -> list[float] | None:
        row = self._db.read_last_row(self._home_position_key)
        if row is None:
            logger.warning(f"DriveHomeCmd: home position row not found in SQLite key {self._home_position_key}")
            return None

        return [float(row["x"]), float(row["y"]), float(row["yaw"])]
        
    def initialize(self):
        self._drive_train.reset_pid()  # Reset PID controller for fresh state at start of movement
        
        home_pose = self._read_home_position()
        if home_pose is None:
            position = self._kalman_estimator.pos
            home_pose = [float(position[0]), float(position[1]), float(self._kalman_estimator.euler[2])]

        current_state = self._kalman_estimator.get_state()
        start_pose = [float(current_state.pos[0]), float(current_state.pos[1]), float(self._kalman_estimator.euler[2])]
        path_matrix = self._path_following.generate_path(start_pose, home_pose)
        self._path_following.set_path(path_matrix)
        self._path_following.start_path_following()
        self._path_following.set_drive_direction(DriveDirection.REVERSE) # set drive direction to reverse for driving back to home position
    
    def execute(self):
        """Poll navigation system and send motor commands."""

        # Get current commands from navigator
        v_cmd, delta_cmd = self._path_following.get_current_commands()

        # Convert to motor commands
        # v_cmd is in m/s, delta_cmd is in radians
        # Convert velocity to percentage (assuming top speed m/s = 1)
        self.speed = int((v_cmd / Constants.rear_motor_top_speed))
        angle = delta_cmd

        logger.debug(f"FollowPathCmd: v_cmd={v_cmd:.2f} m/s, delta_cmd={delta_cmd:.2f} rad -> speed={self.speed}%, angle={angle} rad")

        # Send to motors via DriveTrain subsystem
        self._drive_train.set_speed_angle(self.speed, angle)
    
    def end(self, interrupted):
        self._path_following.stop_path_following()
        self._drive_train.stop()
    
    def is_finished(self):
        return self._path_following.is_at_goal(0.1)