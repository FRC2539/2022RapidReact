from commands2 import CommandBase

import robot


class BackwardConveyorCommand(CommandBase):
    """
    Run the conveyor backward.
    """

    def __init__(self):
        super().__init__()
        self.addRequirements(robot.ballsystem)

    def execute(self):
        robot.ballsystem.backwardConveyor()

    def end(self, interrupted):
        robot.ballsystem.stopConveyor()
