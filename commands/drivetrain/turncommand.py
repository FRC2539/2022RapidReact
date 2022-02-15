from threading import currentThread
from commands2 import CommandBase
from constants import drivetrain as drivetrainConstants
import math

import robot

from wpimath.kinematics import ChassisSpeeds
from wpimath.geometry import Pose2d, Rotation2d, Translation2d


class TurnCommand(CommandBase):
    """
    Turns a certain number of radians, or to a specified angle.

    turnAngle - CCW Positive, radians
    """

    def __init__(
        self,
        turnAngle,
        relative=True,
        turnSpeed=1,
        accelerationRate=4,
        minRotationSpeed=drivetrainConstants.angularSpeedMinimum,
    ):
        super().__init__()

        if relative:
            # sets the angle that the robot will turn and the time it will take to do so
            # (-) accounts for the fact that the robot is counterclockwise positive
            self.turnAngle = -turnAngle
        else:
            rotation = Rotation2d(turnAngle)
            self.turnAngle = (
                robot.drivetrain.getSwervePose().rotation().rotateBy(rotation).radians()
            )

        self.addRequirements(robot.drivetrain)

        # sets how close the robot must be to the target angle to stop the command
        self.radianTolerance = drivetrainConstants.autoTolerance.rotation().radians()

        # sets the max speed the robot should be rotating at
        self.maxRotationSpeed = abs(turnSpeed)

        # sets the acceleration rate and minRotationSpeed based on the values input to the function
        self.accelerationRate = abs(accelerationRate)
        self.minRotationSpeed = abs(minRotationSpeed)

        self.executionNumber = -1

    def initialize(self):
        # saves the original pose of the robot
        self.initialAngle = self.getCurrentAngle()
        self.currentRotationSpeed = math.copysign(self.minRotationSpeed, self.turnAngle)

        # tells the command it is starting to turn and not stopping to turn
        self.accelerating = True
        self.decelerating = False

    def execute(self):
        # sets the current rotation speed based on the trapezoidal speed curve method
        self.executionNumber += 1
        turnChassisSpeed = ChassisSpeeds(0, 0, self.calculateTrapezoidSpeed())
        robot.drivetrain.setChassisSpeeds(turnChassisSpeed)
        if self.executionNumber % 50 == 0:
            print(
                f"{self.getDistanceToTargetAngle()=}\n{self.getDistanceFromInitialPose()=}\n{self.accelerating=}, {self.decelerating=}"
            )

    def isFinished(self):
        # checks if the current angle of the robot is within the tolerance of the wanted angle
        return (
            -self.getDistanceToTargetAngle()
            * math.copysign(1, self.currentRotationSpeed)
            <= self.radianTolerance
        )

    def end(self, interrupted):
        # stops the robot if the command is for some reason halted
        robot.drivetrain.stop()

    def calculateTrapezoidSpeed(self):
        """Returns the speed that the robot should be turning at based on the trapezoidal velocity curve.
        Also adjusts the velocity that the robot is rotating at. Is an edit in place function."""

        # stops the robot from accelerating if it has reached the target speed
        if (
            abs(self.currentRotationSpeed) >= self.maxRotationSpeed
            and self.accelerating
        ):
            self.currentRotationSpeed = math.copysign(
                self.maxRotationSpeed, self.currentRotationSpeed
            )
            self.accelerating = False

        # makes the robot start to deccelerate if it is close enough to the target
        if (
            abs(self.getDistanceToTargetAngle())
            < abs(self.currentRotationSpeed)
            * 1.75  # NOTE THere is a better way to do this
            # this number is how close to the target it will get before deceleraing
        ):
            self.decelerating = True
            self.accelerating = False

        # accelerates or decellerates the robot
        if self.decelerating:
            self.currentRotationSpeed -= self.accelerationRate * math.copysign(
                0.020, self.currentRotationSpeed
            )
        elif self.accelerating:
            self.currentRotationSpeed += self.accelerationRate * math.copysign(
                0.020, self.currentRotationSpeed
            )

        # clamps the rotation speed
        # NOTE MUST BE LAST
        self.currentRotationSpeed = math.copysign(
            min(
                max(abs(self.currentRotationSpeed), self.minRotationSpeed),
                self.maxRotationSpeed,
            ),
            self.currentRotationSpeed,
        )

        return self.currentRotationSpeed

    def getDistanceToTargetAngle(self):
        """Returns the distance between the robot angle and the target angle"""
        currentRotationRadians = self.getCurrentAngle()
        initialRotationRadians = self.initialAngle
        return currentRotationRadians - initialRotationRadians - self.turnAngle

    def getDistanceFromInitialPose(self):
        """Returns the distance between the robot angle and the initial angle"""
        currentRotationRadians = self.getCurrentAngle()
        initialRotationRadians = self.initialAngle
        return currentRotationRadians - initialRotationRadians

    def getCurrentAngle(self):
        return robot.drivetrain.navX.getAngle() / 180 * math.pi
