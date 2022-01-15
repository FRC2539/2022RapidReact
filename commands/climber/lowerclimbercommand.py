from commands2 import CommandBase

import robot


class LowerClimberCommand(CommandBase):
    """
    Lowers the climber, stopping at the lower limit.
    For 2022, this is suitable for manual control or the low bar.
    """

    def __init__(self):
        super().__init__()

        self.addRequirements(robot.climber)

    def execute(self):
        robot.climber.lowerClimber()

    def end(self, interrupted):
        robot.climber.stopClimber()
