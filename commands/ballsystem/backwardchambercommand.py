from commands2 import CommandBase

import robot


class BackwardChamberCommand(CommandBase):
    """
    Run the chamber/launcher backward.
    """

    def __init__(self):
        super().__init__()
        self.addRequirements(robot.ballsystem)

    def execute(self):
        robot.ballsystem.backwardChamber()

    def end(self, interrupted):
        robot.ballsystem.stopChamber()
