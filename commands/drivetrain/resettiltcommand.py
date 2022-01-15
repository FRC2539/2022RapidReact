from commands2 import InstantCommand

import robot


class ResetTiltCommand(InstantCommand):
    """
    Resets the tilt of the robot's gyro.
    """

    def __init__(self):
        super().__init__()

        self.addRequirements(robot.drivetrain)

    def runsWhenDisabled(self):
        return True

    def initialize(self):
        robot.drivetrain.resetTilt()
