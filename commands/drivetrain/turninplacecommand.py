from threading import currentThread
from commands2 import CommandBase
from constants import drivetrain as drivetrainConstants
import math

import robot

from wpimath.kinematics import ChassisSpeeds
from wpimath.geometry import Pose2d, Rotation2d, Translation2d


class TurnInPlaceCommand(CommandBase):
    def __init__(
        self,
        turnAngle,
        time,
        accelerationRate=drivetrainConstants.maxAngularAcceleration,
        minRotationSpeed=0.1,
    ):
        super().__init__()

        self.turnAngle = turnAngle
        self.time = time

        self.addRequirements(robot.drivetrain)

        self.radianTolerance = 0.087

        self.rotationSpeed = turnAngle / time

        self.accelerationRate = accelerationRate
        self.minRotationSpeed = minRotationSpeed

    def initialize(self):
        self.initialPose = robot.drivetrain.getSwervePose()
        self.currentRotationSpeed = 0.1
        self.accelerating = True
        self.decelerating = False

    def execute(self):
        self.calculateTrapezoidSpeed()
        turnChassisSpeed = ChassisSpeeds(0, 0, self.calculateTrapezoidSpeed())
        robot.drivetrain.setChassisSpeeds(turnChassisSpeed)

    def isFinished(self):
        return (
            -self.radianTolerance
            <= self.getDistanceToTargetAngle()
            <= self.radianTolerance
        )

    def end(self, interrupted):
        robot.drivetrain.stop()

    def calculateTrapezoidSpeed(self):
        """Returns the speed that the robot should be turning at."""
        if self.currentRotationSpeed >= self.rotationSpeed:
            self.currentRotationSpeed = self.rotationSpeed
            self.accelerating = False

        if self.getDistanceToTargetAngle() < 0.3:
            self.decelerating = True

        if self.accelerating:
            self.currentRotationSpeed += self.accelerationRate * 0.020

        if self.decelerating:
            self.currentRotationSpeed -= self.accelerationRate * 0.020

        if abs(self.currentRotationSpeed) >= self.rotationSpeed:
            self.currentRotationSpeed = math.copysign(
                self.rotationSpeed, self.currentRotationSpeed
            )

        if abs(self.currentRotationSpeed) <= self.minRotationSpeed:
            self.currentRotationSpeed = math.copysign(
                self.minRotationSpeed, self.currentRotationSpeed
            )

        return self.currentRotationSpeed

    def getDistanceToTargetAngle(self):
        """Returns the distance between the robot angle and the target angle"""
        currentPosition = robot.drivetrain.getSwervePose()
        currentRotationRadians = currentPosition.rotation().radians()
        initialRotationRadians = self.initialPose.rotation().radians()
        return currentRotationRadians - initialRotationRadians - self.turnAngle

    def getDistanceFromInitialPose(self):
        """Returns the distance between the robot angle and the initial angle"""
        currentPosition = robot.drivetrain.getSwervePose()
        currentRotationRadians = currentPosition.rotation().radians()
        initialRotationRadians = self.initialPose.rotation().radians()
        return currentRotationRadians - initialRotationRadians
