from commands2 import InstantCommand

import robot
import constants


class ZeroGyroCommand(InstantCommand):
    """
    Zero the gyro. Counterclockwise is positive.
    This also accounts for the gyro offset (forward on robot vs default forward for navX)
    """

    def __init__(self, angle=0):
        super().__init__()

        self.addRequirements(robot.drivetrain)

        self.offsetAngle = angle + constants.drivetrain.gyroOffset

    def initialize(self):
        robot.drivetrain.resetGyro(self.offsetAngle)
