from structure.commands.Command import Command
from pathlib import Path
import logging

from Robot.Commands.log_data.csvlib import overwrite_csv_with_row
from Robot.subsystems.algorithms.KalmanStateEstimator import KalmanStateEstimator


logger = logging.getLogger(f"{__name__}.RecordHomeCmd")

class RecordHomeCmd(Command):
    def __init__(self):
        super().__init__()
        self._kalman_estimator = KalmanStateEstimator()
        project_root = Path(__file__).resolve().parents[2]
        self._home_position_csv = project_root / "records" / "home_position.csv"
        self._fieldnames = ["x", "y", "yaw"]
        self._write_succeeded = False
        
    def initialize(self):
        current_state = self._kalman_estimator.get_state()
        row = {
            "x": float(current_state.pos[0]),
            "y": float(current_state.pos[1]),
            "yaw": float(self._kalman_estimator.euler[2]),
        }
        self._write_succeeded = overwrite_csv_with_row(
            filename=str(self._home_position_csv),
            fieldnames=self._fieldnames,
            row=row,
        )
        if not self._write_succeeded:
            logger.warning(f"RecordHomeCmd: failed to write home position CSV at {self._home_position_csv}")
    
    def execute(self):
        pass
    
    def end(self, interrupted):
        pass
    
    def is_finished(self):
        return True