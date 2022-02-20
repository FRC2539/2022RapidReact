from commands2 import CommandBase

import robot


class ForwardChamberCommand(CommandBase):
    """
    Run the chamber/launcher forward.
    """

    def __init__(self):
        super().__init__()
        self.addRequirements(robot.ballsystem)

    def execute(self):
        robot.ballsystem.forwardChamber()

    def end(self, interrupted):
        robot.ballsystem.stopChamber()
