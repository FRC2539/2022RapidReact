from commands2 import CommandBase

import robot


class SetIntakeSpeedLimitCommand(CommandBase):
    def __init__(self):
        super().__init__()

    def initialize(self):
        robot.drivetrain.enableIntakeSpeed()

    def end(self, interrupted):
        robot.drivetrain.disableIntakeSpeed()
