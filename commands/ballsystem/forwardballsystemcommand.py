from commands2 import CommandBase

import robot


class ForwardBallSystemCommand(CommandBase):

    """
    Run the conveyor and chamber/launcher forward.
    """

    def __init__(self):
        super().__init__()
        self.addRequirements(robot.ballsystem)

    def execute(self):
        robot.ballsystem.forwardConveyor()
        robot.ballsystem.forwardChamber()

    def end(self, interrupted):
        robot.ballsystem.stopConveyor()
        robot.ballsystem.stopChamber()
