from commands2 import CommandBase

import robot


class ForwardConveyorCommand(CommandBase):
    """
    Run the conveyor forward.
    """

    def __init__(self):
        super().__init__()
        self.addRequirements(robot.ballsystem)

    def execute(self):
        robot.ballsystem.forwardConveyor()

    def end(self, interrupted):
        robot.ballsystem.stopConveyor()
