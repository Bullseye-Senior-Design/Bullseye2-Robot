from Robot.subsystems.HeaderHealerSwitches import HeaderHealerSwitches
from Robot.subsystems.Clutches import Clutches
from Robot.subsystems.DriveTrain import DriveTrain
from Robot.subsystems.PCBLEDs import PCBLEDs
from structure.commands.Command import Command
import time
import math
import logging

logger = logging.getLogger("MotorMovementExampleCmd")
logger.setLevel(level=logging.INFO)

class MotorMovementExampleCmd(Command):
    def __init__(self, drive_train: DriveTrain, clutches: Clutches, header_healer_switches: HeaderHealerSwitches, PCBLEDs: PCBLEDs):
        super().__init__()
        
        self.drive_train = drive_train
        self.clutches = clutches
        self.header_healer_switches = header_healer_switches
        self.pcb_leds = PCBLEDs
        self.add_requirement(self.drive_train)
        self.add_requirement(self.header_healer_switches)
        self.add_requirement(self.clutches)
        self.add_requirement(self.pcb_leds)

        logger.info("MotorMovementExampleCmd initialized with DriveTrain, Clutches, HeaderHealerSwitches, and PCBLEDs")
        
    def initialize(self):
        logger.info("Starting MotorMovementExampleCmd: Cycling motor speed and steering angle while toggling LEDs")
        self.start_time = time.time()
        self.speed = 1
        self.clutches.disengage_clutches()

    def execute(self):
        elapsed_time = time.time() - self.start_time
        # Cycle between -1 and 1 with a period of 10 seconds
        # self.speed = math.sin(elapsed_time * 2 * math.pi / 10.0)
        # self.drive_train.set_speed_angle(self.speed, 0)
        
        # #logger.debug(f"")
        # #logger.debug(f"Front wheel position: {self.drive_train.get_frontwheel_position()}")
        # if self.speed > 0:
        #     self.pcb_leds.set_green_status_led(True)
        #     self.drive_train.engage_backwheel()
        #     self.drive_train.engage_frontwheel()
            
        # else:
        #     self.pcb_leds.set_green_status_led(False)
        #     self.drive_train.disengage_backwheel()
        #     self.drive_train.disengage_frontwheel()
        #     # self.clutches.disengage_clutches()

        angle = math.radians(20)  # Convert 20 degrees to radians

        logger.info(f"Setting speed to {self.speed:.2f} and angle to {math.degrees(angle):.2f} degrees")

        self.drive_train.set_speed_angle(self.speed, angle)
    
    def end(self, interrupted):
        logger.info("Ending MotorMovementExampleCmd: Stopping motors and resetting LEDs")
        self.pcb_leds.set_green_status_led(False)

        self.drive_train.stop()
        self.clutches.disengage_left_clutch()
        self.clutches.disengage_right_clutch()
        self.drive_train.disengage_backwheel()
        self.drive_train.disengage_frontwheel()
    
    def is_finished(self):
        return False