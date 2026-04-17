from structure.commands.Command import Command
import time
import numpy as np
from Robot.subsystems.algorithms.PathFollowing import PathFollowing, DriveDirection
from Robot.subsystems.algorithms.KalmanStateEstimator import KalmanStateEstimator
from Robot.subsystems.DriveTrain import DriveTrain
import logging
from Robot.Constants import Constants
from helpers.SavedPathsHelper import SavedPathsHelper
from Comms.PiCommThread import PiCommThread

logger = logging.getLogger(f"{__name__}.FollowPathCmd")
logger.setLevel(logging.DEBUG)  # Set to DEBUG for detailed output

class FollowPathCmd(Command):
    """Command that uses MPCNavigator to follow a path.

    The command continuously polls the MPC navigation system and sends
    motor commands to MotorControl subsystem. Runs until the command is cancelled.
    """

    def __init__(
        self,
        drive_train: DriveTrain,
        path_following: PathFollowing,
    ):
        """Initialize FollowPathCmd with a saved path.
        
        Args:
            drive_train: DriveTrain subsystem for motor control
            path_following: PathFollowing subsystem for navigation
            path_id: ID of the saved path to follow. If None, retrieves from PiCommThread.
        """
        super().__init__()
        self.drive_train = drive_train
        self.path_following = path_following
        self.add_requirement(drive_train)
        self.add_requirement(path_following)
        
        # Load path using SavedPathsHelper
        self.path_helper = SavedPathsHelper()
        self.is_approaching_boundary = False

        
    def initialize(self):
        """Start path following."""
        self.path_speed, self.path_id = PiCommThread().get_path_data()
        if self.path_id is None or self.path_speed is None:
            logger.warning("No path data received from PiCommThread, using defaults")
            self.path_id = 1  # Default path ID
            self.path_speed = 0.0  # Default speed in m/s

        # Load the path data
        raw_path_data = self.path_helper.load_path_by_id(self.path_id)

        if raw_path_data is None:
            logger.warning(f"Failed to load path {self.path_id}, using empty path")
            self.path_data = np.empty((0, 3), dtype=float)
        else:
            self.path_data = np.array(raw_path_data, dtype=float)
            logger.info(f"Loaded path {self.path_id} with {len(self.path_data)} points")
        
        self._last_update_time = 0.0
        
        self.path_following.set_path(self.path_data)
        self.path_following.set_nominal_speed(self.path_speed)
        self.path_following.set_drive_direction(DriveDirection.FORWARD) # set drive direction to forward for while following a user path
        self.speed = 0.0
        self.is_approaching_boundary = False
        
        self.path_following.start_path_following()
        self._last_update_time = time.time()
        logger.info("FollowPathCmd: Path following initialized")
        self.drive_train.reset_pid()  # Reset PID controller for fresh state at start of movement

    

    def execute(self):
        """Poll navigation system and send motor commands."""

        # Get current commands from navigator
        v_cmd, delta_cmd = self.path_following.get_current_commands()

        # Convert to motor commands
        # v_cmd is in m/s, delta_cmd is in radians
        # Convert velocity to percentage (assuming top speed m/s = 1)
        self.speed = (v_cmd / Constants.rear_motor_top_speed)
        angle = -delta_cmd

        logger.debug(f"FollowPathCmd: v_cmd={v_cmd:.2f} m/s, delta_cmd={delta_cmd:.2f} rad -> speed={self.speed}%, angle={angle} rad")

        # Send to motors via DriveTrain subsystem
        # self.drive_train.set_speed_angle(self.speed, angle)

    def end(self, interrupted):
        """Stop path following and clean up."""
        # Stop navigation
        self.path_following.stop_path_following()
        
        # Stop motors
        self.drive_train.stop()
        
        if self.is_approaching_boundary:
            # TODO - Send an error packet up to the steam deck
            logger.warning("FollowPathCmd ended due to approaching boundary. Stopping robot to prevent collision.")
                

    def is_finished(self):
        """Command runs until cancelled."""
        # self.is_approaching_boundary = self.drive_train.is_approaching_boundary(self.speed)
        
        return self.path_following.is_at_goal(0.1) # or self.is_approaching_boundary
