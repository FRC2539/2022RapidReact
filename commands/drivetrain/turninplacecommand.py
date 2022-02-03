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
        turnSpeed=drivetrainConstants.angularSpeedLimit,
        accelerationRate=drivetrainConstants.maxAngularAcceleration,
        minRotationSpeed=drivetrainConstants.angularSpeedMinimum,
    ):
        super().__init__()

        # sets the angle that the robot will turn and the time it will take to do so
        self.turnAngle = turnAngle
        self.addRequirements(robot.drivetrain)

        # sets how close the robot must be to the target angle to stop the command
        self.radianTolerance = drivetrainConstants.autoTolerance.rotation().radians()

        # gets the speed the robot should be rotating at
        self.maxRotationSpeed = turnSpeed

        # sets the acceleration rate and minRotationSpeed based on the values input to the function
        self.accelerationRate = math.abs(accelerationRate)
        self.minRotationSpeed = math.abs(minRotationSpeed)

    def initialize(self):
        # saves the original pose of the robot
        self.initialPose = robot.drivetrain.getSwervePose()
        self.currentRotationSpeed = self.minRotationSpeed

        # tells the command it is starting to turn and not stopping to turn
        self.accelerating = True
        self.decelerating = False

    def execute(self):
        # sets the current rotation speed based on the trapezoidal speed curve method
        turnChassisSpeed = ChassisSpeeds(0, 0, self.calculateTrapezoidSpeed())
        robot.drivetrain.setChassisSpeeds(turnChassisSpeed)

    def isFinished(self):
        # checks if the current angle of the robot is within the tolerance of the wanted angle
        return (
            -self.radianTolerance
            <= self.getDistanceToTargetAngle()
            <= self.radianTolerance
        )

    def end(self, interrupted):
        # stops the robot if the command is for some reason halted
        robot.drivetrain.stop()

    def calculateTrapezoidSpeed(self):
        """Returns the speed that the robot should be turning at based on the trapezoidal velocity curve."""

        # stops the robot from accelerating if it has reached the target speed
        if (
            math.abs(self.currentRotationSpeed) >= self.maxRotationSpeed
            and self.accelerating
        ):
            self.currentRotationSpeed = math.copysign(
                self.maxRotationSpeed, self.currentRotationSpeed
            )
            self.accelerating = False

        # makes the robot start to deccelerate if it is close enough to the target
        if (
            self.getDistanceToTargetAngle()
            < 0.3  # this number is how close to the target it will get before deceleraing
            and not self.decelerating
            and not self.accelerating
        ):
            self.decelerating = True

            # accelerates or decelerates the robot
            self.currentRotationSpeed += self.accelerationRate * 0.020

        if self.decelerating:
            self.currentRotationSpeed -= self.accelerationRate * 0.020
        elif math.abs(self.currentRotationSpeed) >= self.rotationSpeed:
            self.currentRotationSpeed = math.copysign(
                self.rotationSpeed, self.currentRotationSpeed
            )

        # ensures that the current speed of the robot is never below the minimum rotation speed
        if math.abs(self.currentRotationSpeed) <= self.minRotationSpeed:
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
