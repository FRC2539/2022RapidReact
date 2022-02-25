from commands2 import CommandBase

import robot


class RaiseHoodCommand(CommandBase):
    """
    Manually raise the hood.
    """

    def __init__(self):
        super().__init__()
        self.addRequirements(robot.hood)

    def execute(self):
        # if robot.hood.isUnderMaxAngle():
        robot.hood.rawMove(robot.hood.speed)

    def end(self, interrupted):
        robot.hood.stop()
