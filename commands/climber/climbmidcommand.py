from commands2 import CommandBase

import robot


class ClimbMidCommand(CommandBase):
    def __init__(self):
        super().__init__()

        self.addRequirements(robot.climber)

    def initialize(self):
        pass

    def execute(self):
        pass

    def end(self, interrupted):
        pass
