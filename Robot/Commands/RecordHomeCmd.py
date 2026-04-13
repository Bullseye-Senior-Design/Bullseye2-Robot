from structure.commands.Command import Command
import logging

from helpers.dbConstants import HOME_POSITION_TABLE
from helpers.sqllib import SQLiteFileManager
from Robot.subsystems.algorithms.KalmanStateEstimator import KalmanStateEstimator


logger = logging.getLogger(f"{__name__}.RecordHomeCmd")

class RecordHomeCmd(Command):
    def __init__(self):
        super().__init__()
        self._kalman_estimator = KalmanStateEstimator()
        self._db = SQLiteFileManager()
        self._home_position_key = HOME_POSITION_TABLE
        self._write_succeeded = False
    
    def initialize(self):
        current_state = self._kalman_estimator.get_state()
        row = HOME_POSITION_TABLE.build_row(
            float(current_state.pos[0]),
            float(current_state.pos[1]),
            float(self._kalman_estimator.euler[2]),
        )
        self._write_succeeded = self._db.overwrite_with_row(
            table=self._home_position_key,
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