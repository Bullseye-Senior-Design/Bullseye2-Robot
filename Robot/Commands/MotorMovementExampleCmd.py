from Robot.subsystems.Clutches import Clutches
from Robot.subsystems.DriveTrain import DriveTrain
from structure.commands.Command import Command

class MotorMovementExampleCmd(Command):
    def __init__(self, drive_train: DriveTrain, clutches: Clutches):
        super().__init__()
        
        self.drive_train = drive_train
        self.clutches = clutches
        self.add_requirement(self.drive_train)
        self.add_requirement(self.clutches)

        
    def initialize(self):
        self.drive_train.set_speed_angle(1, 90) 
        
        pass
    
    def execute(self):
        pass
    
    def end(self, interrupted):
        self.drive_train.stop()
        pass
    
    def is_finished(self):
        return False