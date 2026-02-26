from Robot.subsystems.HeaderHealerSwitches import HeaderHealerSwitches
from Robot.subsystems.Clutches import Clutches
from Robot.subsystems.DriveTrain import DriveTrain
from structure.commands.Command import Command

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
        
        if self.header_healer_switches.get_header_switch_triggered():
            self.clutches.engage_left_clutch()
            self.clutches.engage_right_clutch()
            self.drive_train.disengage_backwheel()
            self.drive_train.disengage_frontwheel()
            
        if self.header_healer_switches.get_healer_switch_triggered():
            self.clutches.disengage_left_clutch()
            self.clutches.disengage_right_clutch()
            self.drive_train.engage_backwheel()
            self.drive_train.engage_frontwheel()
            
        pass
    
    def execute(self):
        self.drive_train.set_speed_angle(1, 180) 
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