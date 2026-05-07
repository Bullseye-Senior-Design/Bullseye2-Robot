from structure.commands.Command import Command
from Robot.subsystems.DriveTrain import DriveTrain
from Robot.subsystems.algorithms.ParkingController import ParkingController
from Robot.subsystems.algorithms.KalmanStateEstimator import KalmanStateEstimator
import logging

from helpers.dbConstants import HOME_POSITION_TABLE
from helpers.sqllib import SQLiteFileManager
from Comms.PiCommThread import PiCommThread
import math

logger = logging.getLogger(f"{__name__}.ParkingCmd")
logger.setLevel(logging.INFO)  # Set to DEBUG for detailed output


class ParkingCmd(Command):
    def __init__(self, drive_train : DriveTrain, parking_controller : ParkingController):
        super().__init__()
        self._drive_train = drive_train
        self._parking_controller = parking_controller
        self._kalman_estimator = KalmanStateEstimator()
        self._db = SQLiteFileManager()
        self._home_position_key = HOME_POSITION_TABLE
        self.add_requirement(drive_train)
        self.is_approaching_boundary = False
    
    def _read_home_position(self) -> list[float] | None:
        row = self._db.read_last_row(self._home_position_key)
        if row is None:
            logger.warning(f"DriveHomeCmd: home position row not found in SQLite key {self._home_position_key}")
            return None
        return [float(row["x"]), float(row["y"]), float(row["yaw"])]

        
    def initialize(self):
        self.home_pose = self._read_home_position()
        self.speed = 0.0
        self.is_approaching_boundary = False
        self._drive_train.reset_pid()  # Reset PID controller for fresh state at start of movement
        self._drive_train.engage_backwheel()
        self._drive_train.engage_frontwheel()
        self._drive_train.clutches.engage_clutches()

        if self.home_pose is None:
            position = self._kalman_estimator.pos
            self.home_pose = [float(position[0]), float(position[1]), float(self._kalman_estimator.euler[2])]
    
    def execute(self):
        current_pos = self._kalman_estimator.get_robot_pose()
        speed, angle = self._parking_controller.compute_commands(current_pos, self.home_pose)
        self.speed = speed
        logger.debug(f"ParkingCmd: speed={speed}, angle={angle}")
        self._drive_train.set_speed_angle(speed, angle)
            
    def end(self, interrupted):
        self._drive_train.stop()
        logger.info("ParkingCmd: Stopped drive train at end of command.")
        PiCommThread().send_route_finished("Robot has parked at home position.")
        
        if self.is_approaching_boundary:
            #TODO - Send an error packet up to the steam deck
            pass
            # logger.warning("ParkingCmd ended due to approaching boundary. Stopping robot to prevent collision.")
    
    def is_finished(self):
        current_pos = self._kalman_estimator.get_robot_pose()
        
        self.is_approaching_boundary = self._drive_train.is_approaching_boundary(self.speed)
        return self._parking_controller.is_at_goal(current_pos, self.home_pose, 0.3, math.radians(5)) # or self.is_approaching_boundary