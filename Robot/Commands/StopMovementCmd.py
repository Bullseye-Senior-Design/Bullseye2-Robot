from Robot.subsystems.DriveTrain import DriveTrain
from structure.commands.Command import Command

class TemplateCommand(Command):
    def __init__(self, drive_train: DriveTrain):
        super().__init__()
        self._drive_train = drive_train
        self.add_requirement(drive_train)
        
    def initialize(self):
        self._drive_train.stop()
    
    def execute(self):
        pass
    
    def end(self, interrupted):
        pass
    
    def is_finished(self):
        return False