from Robot.subsystems.HeaderHealerSwitches import HeaderHealerSwitches
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
from helpers.sqllib import SQLiteFileManager
from helpers.dbConstants import MPC_COMMANDS_TABLE
import math

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
        header_healer_limit_switches: HeaderHealerSwitches,
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
        self.header_healer_limit_switches = header_healer_limit_switches
        self.path_following = path_following
        self.add_requirement(drive_train)
        self.add_requirement(header_healer_limit_switches)
        self.add_requirement(path_following)
        
        # Load path using SavedPathsHelper
        self.path_helper = SavedPathsHelper()
        self._pi_comm_thread = PiCommThread()
        self.is_approaching_boundary = False
        
        # Initialize SQL logging
        self.db_manager = SQLiteFileManager()
        self.db_manager.setup_file(MPC_COMMANDS_TABLE)

        
    def initialize(self):
        """Start path following."""
        self.path_speed, self.path_id = self._pi_comm_thread.get_path_data()
        if self.path_id is None or self.path_speed is None:
            logger.warning("No path data received from PiCommThread, using defaults")
            self.path_id = 1  # Default path ID
            self.path_speed = 0.0  # Default speed in m/s

        # Reset the MPC command log for each newly driven path.
        self.db_manager.setup_file(MPC_COMMANDS_TABLE, replace_existing=True)
        logger.info(f"Reset MPC command log for path {self.path_id}")

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
        logger.debug(f"FollowPathCmd: Starting to follow path {self.path_id} at speed {self.path_speed} -1/1")
        self.path_following.set_drive_direction(DriveDirection.FORWARD) # set drive direction to forward for while following a user path
        self.speed = 0.0
        self.is_approaching_boundary = False
        
        self.path_following.start_path_following()
        self._last_update_time = time.time()
        logger.info("FollowPathCmd: Path following initialized")
        self.drive_train.reset_pid()  # Reset PID controller for fresh state at start of movement
        self.drive_train.engage_backwheel()
        self.drive_train.engage_frontwheel()
        self.drive_train.clutches.engage_clutches()

    

    def execute(self):
        """Poll navigation system and send motor commands."""

        # Get current commands from navigator
        v_cmd, delta_cmd = self.path_following.get_current_commands()

        # Convert to motor commands
        # v_cmd is in m/s, delta_cmd is in radians
        # Convert velocity to percentage (assuming top speed m/s = rear_motor_top_speed)
        self.speed = (v_cmd / Constants.rear_motor_top_speed)
        angle = delta_cmd # Negating seems to required to match the direction of steering commands with the robot's response. This may be due to differences in coordinate conventions between the path following algorithm and the robot's control system.
        
        logger.debug(f"Robot position: x,y,yaw={KalmanStateEstimator().get_robot_pose()}")
        logger.debug(f"FollowPathCmd: v_cmd={v_cmd:.2f} m/s, delta_cmd={delta_cmd:.2f} rad -> speed={self.speed}%, angle={angle} rad")

        # Log MPC command to database
        steering_angle_deg = math.degrees(angle)
        mpc_command = MPC_COMMANDS_TABLE.build_row(
            timestamp=time.time(),
            path_id=self.path_id,
            velocity_cmd_mps=v_cmd,
            steering_angle_cmd_rad=angle,
            steering_angle_cmd_deg=steering_angle_deg,
        )
        self.db_manager.write_row(MPC_COMMANDS_TABLE, mpc_command)

        # Send to motors via DriveTrain subsystem
        self.drive_train.set_speed_angle(self.speed, angle)

    def end(self, interrupted):
        """Stop path following and clean up."""
        # Stop navigation
        self.path_following.stop_path_following()
        # Stop motors
        self.drive_train.stop()
        
        message = "Path following completed successfully." if not interrupted else "Path following interrupted."

        if self.is_approaching_boundary:
            # TODO - Send an error packet up to the steam deck
            logger.warning("FollowPathCmd ended due to approaching boundary. Stopping robot to prevent collision.")
            message = "Path following stopped: approaching boundary. Robot stopped to prevent collision."

        is_switch_triggered = self.header_healer_limit_switches.get_header_switch_triggered() or self.header_healer_limit_switches.get_healer_switch_triggered()

        if is_switch_triggered:
            logger.info("FollowPathCmd ended due to header or healer switch trigger. Stopping robot.")
            message = "Path following stopped: header or healer switch triggered. Robot stopped."

        self._pi_comm_thread.send_route_finished(message)
        logger.info(f"FollowPathCmd set message to steam deck: {message}")

    def is_finished(self):
        """Command runs until cancelled."""
        self.is_approaching_boundary = self.drive_train.is_approaching_boundary(self.speed)
        
        is_switch_triggered = self.header_healer_limit_switches.get_header_switch_triggered() or self.header_healer_limit_switches.get_healer_switch_triggered()


        return self.path_following.is_at_goal(Constants().is_at_end_tolerance) or self.is_approaching_boundary or is_switch_triggered
