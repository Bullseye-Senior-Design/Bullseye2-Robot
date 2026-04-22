from Robot.subsystems.DriveTrain import DriveTrain
from Robot.subsystems.subsystemChildren.Clutches import Clutches
from structure.commands.Command import Command

class StopMovementCmd(Command):
    def __init__(self, drive_train: DriveTrain):
        super().__init__()
        self._drive_train = drive_train
        self.add_requirement(drive_train)
        
    def initialize(self):
        self._drive_train.clutches.disengage_clutches()
        self._drive_train.disengage_backwheel()
        self._drive_train.disengage_frontwheel()
        self._drive_train.stop()
    
    def execute(self):
        pass
    
    def end(self, interrupted):
        pass
    
    def is_finished(self):
        return True