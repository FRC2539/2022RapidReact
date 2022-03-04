from commands2 import CommandBase

import robot
import constants
import math

from wpimath.kinematics import ChassisSpeeds
from wpimath.geometry import Pose2d


class LimelightAngleLockCommand(CommandBase):
    """
    Use the limelight to orient the robot towards the target.
    """

    def __init__(self):
        super().__init__()
        self.addRequirements(robot.drivetrain)

        # Use the drive theta controller from the drivetrain subsystem
        # self.pidController = robot.drivetrain.driveThetaController

    def initialize(self):
        pass
        # Reset the pid controller
        # self.pidController.reset(self.getMeasurement())
        # self.pidController.setGoal(0)
        # self.pidController.setTolerance(constants.limelight.drivetrainAngleTolerance)

    def execute(self):
        # Update the drivetrain to have it match the desired rotational velocity (omega)
        # Note: the desired rotational velocity comes from the pid controller
        robot.drivetrain.move(0, 0, self.calculateDesiredRotation())

    def end(self, interrupted):
        robot.drivetrain.stop()

    def calculateDesiredRotation(self):
        """
        See the LimelightAngleLockCommand for further documentation.

        Returns radians/second
        """

        xOffsetP = 0.05

        xOffset = robot.limelight.getX()  # Returns an angle

        try:
            xPercentError = xOffset * xOffsetP  # This value is found experimentally
        except (TypeError):
            xPercentError = 0
            print("\nERROR: Limelight is broken/unplugged \n")

        if abs(xPercentError) > 0.25:
            xPercentError = math.copysign(0.15, xPercentError)

        return xPercentError
