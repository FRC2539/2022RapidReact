from commands2 import CommandBase

import robot


class LowerHoodCommand(CommandBase):
    """
    Manually lower the hood.
    """

    def __init__(self):
        super().__init__()
        self.addRequirements(robot.hood)

    def execute(self):
        robot.hood.move(-robot.hood.speed)

    def end(self, interrupted):
        robot.hood.stop()
