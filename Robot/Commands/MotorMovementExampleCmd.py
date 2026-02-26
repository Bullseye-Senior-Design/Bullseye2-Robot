from Robot.subsystems.DriveTrain import DriveTrain
from structure.commands.Command import Command

class MotorMovementExampleCmd(Command):
    def __init__(self, drive_train: DriveTrain):
        super().__init__()
        
        self.drive_train = drive_train
        self.add_requirement(self.drive_train)

        
    def initialize(self):
        
        pass
    
    def execute(self):
        self.drive_train.set_speed_angle(1, 180) 
        pass
    
    def end(self, interrupted):
        self.drive_train.stop(0, 0)
        pass
    
    def is_finished(self):
        return False