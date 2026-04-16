from Robot.subsystems.DriveTrain import DriveTrain
from Robot.subsystems.Clutches import Clutches
from structure.commands.Command import Command

class StopMovementCmd(Command):
    def __init__(self, drive_train: DriveTrain, clutches: Clutches):
        super().__init__()
        self._drive_train = drive_train
        self._clutches = clutches
        self.add_requirement(clutches)
        self.add_requirement(drive_train)
        
    def initialize(self):
        self._clutches.disengage_clutches()
        self._drive_train.stop()
    
    def execute(self):
        pass
    
    def end(self, interrupted):
        pass
    
    def is_finished(self):
        return False