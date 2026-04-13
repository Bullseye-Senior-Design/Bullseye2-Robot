from structure.commands.Command import Command
from Robot.subsystems.DriveTrain import DriveTrain
from Robot.subsystems.algorithms.ParkingController import ParkingController
from Robot.subsystems.algorithms.KalmanStateEstimator import KalmanStateEstimator
import logging
from Robot.Constants import Constants

from helpers.sqllib import SQLiteFileManager

logger = logging.getLogger(f"{__name__}.ParkingCmd")


class ParkingCmd(Command):
    def __init__(self, drive_train : DriveTrain, parking_controller : ParkingController):
        super().__init__()
        self._drive_train = drive_train
        self._parking_controller = parking_controller
        self._kalman_estimator = KalmanStateEstimator()
        self._db = SQLiteFileManager()
        self._home_position_key = Constants.records_directory / "home_position"
        self.add_requirement(drive_train)
    
    def _read_home_position(self) -> list[float] | None:
        row = self._db.read_last_row(str(self._home_position_key))
        if row is None:
            logger.warning(f"DriveHomeCmd: home position row not found in SQLite key {self._home_position_key}")
            return None

        return [float(row["x"]), float(row["y"]), float(row["yaw"])]
        
    def initialize(self):
        self.home_pose = self._read_home_position()
        if self.home_pose is None:
            position = self._kalman_estimator.pos
            self.home_pose = [float(position[0]), float(position[1]), float(self._kalman_estimator.euler[2])]
    
    def execute(self):
        current_state = self._kalman_estimator.get_state()
        current_pos = [float(current_state.pos[0]), float(current_state.pos[1]), float(self._kalman_estimator.euler[2])]
        self._parking_controller.compute_commands(current_pos, self.home_pose)
            
    def end(self, interrupted):
        self._drive_train.stop()
    
    def is_finished(self):
        current_state = self._kalman_estimator.get_state()
        current_pos = [float(current_state.pos[0]), float(current_state.pos[1]), float(self._kalman_estimator.euler[2])]
        return self._parking_controller.is_at_goal(current_pos, self.home_pose)