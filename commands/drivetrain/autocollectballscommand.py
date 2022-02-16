from commands2 import CommandBase

import robot

import constants

from wpimath.geometry import Translation2d, Rotation2d, Pose2d
from wpilib import Timer
from wpimath.kinematics import ChassisSpeeds
import constants
import math


class AutoCollectBallsCommand(CommandBase):
    def __init__(self):
        super().__init__()
        self.addRequirements(robot.drivetrain)

        self.reactionSpeed = 0.03
        self.maxRotationSpeed = constants.drivetrain.angularSpeedLimit / 4
        self.minRotationSpeed = constants.drivetrain.angularSpeedMinimum

        self.maxLinearSpeed = constants.drivetrain.speedLimit / 6

        self.turningRadianTolerance = 0.2
        self.movingRadianTolerance = 0.4

        self.timer = Timer()

    def initialize(self):
        robot.intake.intakeBalls()
        self.timer.reset()

    def execute(self):

        robot.drivetrain.setChassisSpeeds(
            ChassisSpeeds(-self.calcForwardVelocity(), 0, self.calcRotationSpeed())
        )

        print(
            f"{robot.ml.getX()=}, {self.getXNormalized()=}, {self.calcRotationSpeed()}"
        )

    def end(self, interrupted):
        robot.drivetrain.stop()
        robot.intake.stop()

    def calcForwardVelocity(self):
        if robot.ml.isTargetAcquired():
            self.timer.reset()
            self.timer.start()

        if not robot.ml.isTargetAcquired() and not self.timer.hasElapsed(0.2):
            return self.maxLinearSpeed

        velocity = 0

        if abs(self.getXAngle()) <= self.movingRadianTolerance:
            velocity = self.maxLinearSpeed

        return velocity

    def calcRotationSpeed(self):
        if not robot.ml.isTargetAcquired():
            return 0

        velocity = self.getXNormalized() * self.reactionSpeed

        absVel = abs(velocity)

        absVel = max(absVel, self.minRotationSpeed)
        absVel = min(absVel, self.maxRotationSpeed)

        if abs(self.getXAngle()) <= self.turningRadianTolerance:
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

    def getYNormalized(self):
        """Returns a value between -1 (left) and 1 (right) for where the ball is on the x axis"""
        yPosition = robot.ml.getY()
        yResolution = robot.ml.getResY()

        # normalizes and then moves
        yNormalized = -(yPosition / yResolution * 2 - 1)

        return yNormalized

    def getXAngle(self):
        """calculates the angle of the ball to the robot x"""
        horizontalAngle = -self.getXNormalized() / constants.ml.horizontalFieldOfView
        return horizontalAngle  # counter clockwise positive
