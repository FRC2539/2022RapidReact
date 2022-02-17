from commands2 import CommandBase

import robot


class EnableLimelightLockCommand(CommandBase):
    def __init__(self):
        super().__init__()

    def initialize(self):
        robot.drivetrain.enableLimelightLock()

    def end(self, interrupted):
        robot.drivetrain.disableLimelightLock()
