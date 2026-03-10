from Robot.subsystems.HeaderHealerSwitches import HeaderHealerSwitches
from Robot.subsystems.Clutches import Clutches
from Robot.subsystems.DriveTrain import DriveTrain
from structure.commands.Command import Command
import time
import math
import logging

logger = logging.getLogger("MotorMovementExampleCmd")
logger.setLevel(level=logging.INFO)

class MotorMovementExampleCmd(Command):
    def __init__(self, drive_train: DriveTrain, clutches: Clutches, header_healer_switches: HeaderHealerSwitches):
        super().__init__()
        
        self.drive_train = drive_train
        self.clutches = clutches
        self.header_healer_switches = header_healer_switches
        self.add_requirement(self.drive_train)
        self.add_requirement(self.header_healer_switches)
        self.add_requirement(self.clutches)
        
    def initialize(self):
        self.start_time = time.time()
        self.speed = 1
        self.clutches.set_green_status_led(True)
        self.clutches.set_red_status_led(False)
        
            
        pass
    
    def execute(self):
        elapsed_time = time.time() - self.start_time
        # Cycle between -1 and 1 with a period of 10 seconds
        self.speed = math.sin(elapsed_time * 2 * math.pi / 10.0)
        self.drive_train.set_speed_angle(self.speed, 180)
        #logger.debug(f"")
        #logger.debug(f"Front wheel position: {self.drive_train.get_frontwheel_position()}")
        self.drive_train.engage_backwheel()
        self.drive_train.engage_frontwheel()
        self.clutches.engage_clutches()
        pass
    
    def end(self, interrupted):
        self.drive_train.stop()
        self.clutches.disengage_left_clutch()
        self.clutches.disengage_right_clutch()
        self.drive_train.disengage_backwheel()
        self.drive_train.disengage_frontwheel()
        pass
    
    def is_finished(self):
        return False