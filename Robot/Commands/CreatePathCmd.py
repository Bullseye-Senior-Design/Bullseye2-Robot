import logging
from typing import Callable
import csv
from pathlib import Path
from uuid import uuid4

from Comms.PiCommThread import PiCommThread
from Robot.subsystems.algorithms.PathCreation import PathCreation
from Robot.subsystems.algorithms.KalmanStateEstimator import KalmanStateEstimator
from structure.commands.Command import Command

logger = logging.getLogger(f"{__name__}.DriveTrain")
logger.setLevel(logging.DEBUG)

class CreatePathCmd(Command):
    def __init__(self, path_creation: PathCreation, comm_thread: PiCommThread, exit_button: Callable[[], bool]):
        super().__init__()
        self._exit_button = exit_button
        self._path_creation = path_creation
        self._comm_thread = comm_thread
        self.add_requirement(path_creation)

    def initialize(self):
        logger.debug("Initializing CreatePathCmd")

        self._saved_file_path = None

        self.kf = KalmanStateEstimator()

        self._path_creation.start_path_creation()
    
    def execute(self):        
        self._path_creation.add_path_point()
    
    def end(self, interrupted):
        logger.debug("Ending CreatePathCmd")

        path_id = self._path_creation.stop_path_creation()
        if path_id is not None:
            self._comm_thread.send_new_path_data(path_id)
    
    def is_finished(self):
        return self._exit_button()