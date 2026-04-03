from Robot.subsystems.Clutches import Clutches
from Robot.subsystems.DriveTrain import DriveTrain
from structure.commands.Command import Command
from typing import Callable
import logging
from Robot.MathUtil import MathUtil
from Robot.Constants import Constants

logger = logging.getLogger(f"{__name__}.DriveTrain")
logger.setLevel(logging.INFO)  # Set to DEBUG for detailed output

class DefaultMovementCmd(Command):
    def __init__(self, drive_train: DriveTrain, clutches: Clutches, throttle_supplier: Callable[[], float], steering_supplier: Callable[[], float]):
        """
        Args:
            drive_train: The DriveTrain subsystem
            clutches: The Clutches subsystem
            throttle_supplier: Function that returns current throttle value (-1.0 to 1.0)
            steering_supplier: Function that returns current steering value (-1.0 to 1.0)
        """
        super().__init__()
        self._drive_train = drive_train
        self._clutches = clutches
        self._throttle_supplier = throttle_supplier
        self._steering_supplier = steering_supplier
        self.add_requirement(drive_train)

    def initialize(self):
        """Called once when the command is first scheduled"""
        self._drive_train.engage_backwheel()  # Ensure backwheel is engaged for movement
        self._drive_train.engage_frontwheel()  # Ensure frontwheel is engaged for movement
        self._clutches.engage_clutches()  # Disengage clutches to allow movement
        self._drive_train.stop()
    
    def execute(self):
        """Called repeatedly while the command is scheduled"""
        # Get current values from suppliers (like getting values through references)
        throttle = self._throttle_supplier()
        steering = self._steering_supplier()

        
        # Convert steering from [-1, 1] to radians [steering_angle_limit, steering_angle_limit]
        # steering=-1.0 -> -steering_angle_limit
        # steering=0.0  -> 0
        # steering=1.0  -> +steering_angle_limit
        angle = MathUtil.map(steering, -1.0, 1.0, -Constants.steering_angle_limit_rads, Constants.steering_angle_limit_rads)
        
        logger.debug(f"commanding throttle {throttle} and angle {angle}")

        # Send to drivetrain
        self._drive_train.set_speed_angle(throttle, angle)
    
    def end(self, interrupted):
        """Called once when the command ends"""
        # Stop the drivetrain when command ends
        self._drive_train.stop()
    
    def is_finished(self):
        """This command runs continuously until interrupted"""
        return False