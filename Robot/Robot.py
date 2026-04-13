from structure.CommandRunner import CommandRunner
from Robot.RobotContainer import RobotContainer
import logging

logger = logging.getLogger(f"{__name__}.Robot")
logger.setLevel(logging.INFO)  # Set to INFO for high-level robot events

class Robot:    
    def __init__(self):
        self.command_runner = CommandRunner()
        self.robot_container = RobotContainer()
    
    def robot_init(self):
        self.command_runner.turn_on()
        self.robot_container.start_robot()
        self.robot_container.begin_data_log()
    
    def robot_periodic(self):
        self.command_runner.run_commands()
        logger.debug(f"Commands scheduled: {self.command_runner.commands}")
        # logger.debug(f"Command Event Loop: {self.command_runner.commands_to_schedule}")
    
    def teleop_init(self):
        self.robot_container.start_teleop()
        pass
            
    def teleop_periodic(self):
        pass

    def path_creation_init(self):
        self.robot_container.start_path_creation()

    def path_creation_periodic(self):
        
        pass

    def return_to_home_init(self):
        self.robot_container.start_parking()

    def return_to_home_periodic(self):
        pass

    def autonomous_init(self):
        self.robot_container.start_path_following()

    def autonomous_periodic(self):
        pass

    def test_init(self):
        pass
    
    def test_periodic(self):
        pass
    
    def disabled_init(self):
        self.robot_container.disabled_init()
    
    def shutdown(self):
        self.robot_container.shutdown()