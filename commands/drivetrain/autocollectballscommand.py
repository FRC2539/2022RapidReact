from commands2 import CommandBase

import robot

from constants import drivetrain as drivetrainConstants

from wpimath.geometry import Translation2d, Rotation2d, Pose2d
from wpimath.kinematics import ChassisSpeeds
import constants
import math


class AutoCollectBallsCommand(CommandBase):
    def __init__(self):
        super().__init__()
        self.addRequirements(robot.drivetrain)

        self.reactionSpeed = 0.5
        self.maxRotationSpeed = drivetrainConstants.angularSpeedLimit / 4
        self.minRotationSpeed = 0  # drivetrainConstants.angularSpeedMinimum

        self.radianTolerance = 0.2

    def initialize(self):
        pass

    def execute(self):
        robot.drivetrain.setChassisSpeeds(
            ChassisSpeeds(self.calcForwardVelocity(), 0, self.calcRotationSpeed())
        )
        print(
            f"{robot.ml.getX()=}, {self.getXNormalized()=}, {self.calcRotationSpeed()}"
        )

    def end(self, interrupted):
        robot.drivetrain.stop()

    def calcForwardVelocity(self):
        return 0

    def calcRotationSpeed(self):
        if not robot.ml.isTargetAcquired():
            return 0

        velocity = self.getXNormalized() * self.reactionSpeed

        absVel = abs(velocity)

        absVel = max(absVel, self.minRotationSpeed)
        absVel = min(absVel, self.maxRotationSpeed)

        if abs(self.getXNormalized()) <= self.radianTolerance:
            absVel = 0

        velocity = math.copysign(absVel, velocity)

        return velocity

    def getXNormalized(self):
        """Returns a value between -1 (left) and 1 (right) for where the ball is on the x axis"""
        xPosition = robot.ml.getX()
        xResolution = robot.ml.getResX()

        # normalizes and then moves
        xNormalized = xPosition / xResolution * 2 - 1

        return xNormalized
