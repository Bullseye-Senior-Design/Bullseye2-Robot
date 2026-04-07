import logging
from typing import Callable
import csv
from pathlib import Path
from uuid import uuid4

from Robot.subsystems.algorithms.PathCreation import PathCreation
from Robot.subsystems.algorithms.KalmanStateEstimator import KalmanStateEstimator
from structure.commands.Command import Command

logger = logging.getLogger(f"{__name__}.DriveTrain")
logger.setLevel(logging.INFO)

class CreatePathCmd(Command):
    def __init__(self, path_creation: PathCreation, exit_button: Callable[[], bool]):
        super().__init__()
        self._exit_button = exit_button
        self._path_creation = path_creation
        self.add_requirement(path_creation)

    def initialize(self):
        self._saved_file_path = None

        self.kf = KalmanStateEstimator()

        self._path_creation.start_path_creation()
    
    def execute(self):        
        self._path_creation.add_path_point()
    
    def end(self, interrupted):
        self._path_creation.stop_path_creation()
    
    def is_finished(self):
        return self._exit_button()