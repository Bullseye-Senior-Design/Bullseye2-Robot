from structure.commands.Command import Command
import logging

from Robot.Commands.helpers.sqllib import SQLiteFileManager
from Robot.Constants import Constants
from Robot.subsystems.algorithms.KalmanStateEstimator import KalmanStateEstimator


logger = logging.getLogger(f"{__name__}.RecordHomeCmd")

class RecordHomeCmd(Command):
    def __init__(self):
        super().__init__()
        self._kalman_estimator = KalmanStateEstimator()
        self._db = SQLiteFileManager()
        self._home_position_key = Constants.records_directory / "home_position"
        self._fieldnames = ["x", "y", "yaw"]
        self._write_succeeded = False
        
    def initialize(self):
        current_state = self._kalman_estimator.get_state()
        row = {
            "x": float(current_state.pos[0]),
            "y": float(current_state.pos[1]),
            "yaw": float(self._kalman_estimator.euler[2]),
        }
        self._write_succeeded = self._db.overwrite_with_row(
            filepath=str(self._home_position_key),
            fieldnames=self._fieldnames,
            row=row,
        )
        if not self._write_succeeded:
            logger.warning(f"RecordHomeCmd: failed to write home position row in SQLite key {self._home_position_key}")
    
    def execute(self):
        pass
    
    def end(self, interrupted):
        pass
    
    def is_finished(self):
        return True