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

        if not robot.ballsystem.isChamberBallPresent():
            robot.ballsystem.forwardChamber()
        else:
            robot.ballsystem.stopChamber()

    def end(self, interrupted):
        robot.ballsystem.stopConveyor()
        robot.ballsystem.stopChamber()
