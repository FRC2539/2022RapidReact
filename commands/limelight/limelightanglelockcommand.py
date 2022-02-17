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
        self.pidController = robot.drivetrain.driveThetaController

    def initialize(self):
        # Reset the pid controller
        self.pidController.reset(self.getMeasurement())
        self.pidController.setGoal(0)
        self.pidController.setTolerance(constants.limelight.drivetrainAngleTolerance)

    def execute(self):
        # Update the drivetrain to have it match the desired rotational velocity (omega)
        # Note: the desired rotational velocity comes from the pid controller
        robot.drivetrain.setChassisSpeeds(
            ChassisSpeeds(0, 0, self.pidController.calculate(self.getMeasurement()))
        )

    def isFinished(self):
        return (
            self.pidController.atGoal() or not robot.limelight.isVisionTargetDetected()
        )

    def end(self, interrupted):
        robot.drivetrain.stop()

    def getMeasurement(self):
        return -1 * robot.limelight.estimateShooterPose().rotation().radians()
