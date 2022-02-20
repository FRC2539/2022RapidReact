from commands2 import CommandBase

import robot


class BackwardBallSystemCommand(CommandBase):
    """
    Run the conveyor and chamber/launcher backward.
    """

    def __init__(self):
        super().__init__()
        self.addRequirements(robot.ballsystem)

    def execute(self):
        robot.ballsystem.backwardConveyor()
        robot.ballsystem.backwardChamber()

    def end(self, interrupted):
        robot.ballsystem.stopConveyor()
        robot.ballsystem.stopChamber()
