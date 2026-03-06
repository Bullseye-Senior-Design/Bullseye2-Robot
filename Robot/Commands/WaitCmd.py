from structure.commands.Command import Command
import time

class WaitCmd(Command):
    def __init__(self, time):
        super().__init__()
        self.time = time

    def initialize(self):
        self.start_time = time.time()
    
    def execute(self):
        pass
    
    def end(self, interrupted):
        pass
    
    def is_finished(self):
        return time.time() - self.start_time >= self.time