from Robot.subsystems.DriveTrain import DriveTrain
from structure.commands.Command import Command
from typing import Callable
import logging

logger = logging.getLogger(f"{__name__}.DriveTrain")
logger.setLevel(logging.INFO)  # Set to DEBUG for detailed output

class DefaultMovementCmd(Command):
    def __init__(self, drive_train: DriveTrain, throttle_supplier: Callable[[], float], steering_supplier: Callable[[], float]):
        """
        Args:
            drive_train: The DriveTrain subsystem
            throttle_supplier: Function that returns current throttle value (-1.0 to 1.0)
            steering_supplier: Function that returns current steering value (-1.0 to 1.0)
        """
        super().__init__()
        self._drive_train = drive_train
        self._throttle_supplier = throttle_supplier
        self._steering_supplier = steering_supplier
        self.add_requirement(drive_train)

    def initialize(self):
        """Called once when the command is first scheduled"""
        self._drive_train.engage_backwheel()  # Ensure backwheel is engaged for movement
        self._drive_train.engage_frontwheel()  # Ensure frontwheel is engaged for movement
        self._drive_train.stop()
    
    def execute(self):
        """Called repeatedly while the command is scheduled"""
        # Get current values from suppliers (like getting values through references)
        throttle = self._throttle_supplier()
        steering = self._steering_supplier()

        
        # Convert steering to angle (0 to 180 degrees)
        # steering=-1.0 -> angle=180 (full left)
        # steering=0 -> angle=90 (straight)
        # steering=1.0 -> angle=0 (full right)
        angle = -steering
        #angle = 90 - (steering * 90)

        #angle = max(0, min(180, 90))  # Clamp to 0-180
        
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