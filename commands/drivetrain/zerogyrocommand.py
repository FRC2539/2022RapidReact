from commands2 import InstantCommand

import robot


class ZeroGyroCommand(InstantCommand):
    def __init__(self, driveOrientation=True):
        super().__init__()

        self.addRequirements(robot.drivetrain)

        self.offsetAngle = 180 if driveOrientation else 0

    def initialize(self):
        robot.drivetrain.resetGyro(self.offsetAngle)
