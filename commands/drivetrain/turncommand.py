from commands2 import CommandBase

import robot
import constants

import math

from wpimath.geometry import Rotation2d, Translation2d, Transform2d

from wpimath.kinematics import SwerveModuleState

from wpilib.controller import PIDController

import constants


class TurnCommand(CommandBase):
    """Allows autonomous turning using the drive base encoders."""

    # TODO use intermediate points and follow, that should fix the problems
    # To clarify, equally spaced points to create a path. In this case,
    # those are equally spaced angles between the current angle and the target

    def __init__(self, degrees, speedPercent=0.3, tolerance=3):
        super().__init__()

        self.degrees = degrees
        self.tolerance = tolerance

        self.driveController = robot.drivetrain.driveController

        self.addRequirements(robot.drivetrain)

        self.speedLimit = constants.drivetrain.speedLimit * speedPercent
        self.wheelAngles = [
            Rotation2d.fromDegrees(angle) for angle in [135, 45, 225, 315]
        ]

        self.pidController = PIDController(0.1, 0.03, 0.003)

    def initialize(self):
        # Create a transform object to transform the current pose by
        self.transformToAngle = Transform2d(
            Translation2d(0, 0), Rotation2d.fromDegrees(self.degrees)
        )

        robot.drivetrain.setModuleProfiles(1, turn=False)

        # Get the current state of the robot
        self.currentPose = robot.drivetrain.getSwervePose()
        self.currentAngle = self.currentPose.rotation()

        # Convert the angle we are given to a pose
        self.desiredPose = self.currentPose.transformBy(self.transformToAngle)
        self.desiredAngle = self.desiredPose.rotation().degrees()

        self.pidController.setSetpoint(0)

        # Set the modules to the correct angle
        targetModuleStates = [SwerveModuleState(0, angle) for angle in self.wheelAngles]
        robot.drivetrain.setModuleStates(targetModuleStates)

        robot.drivetrain.addAutoPeriodicFunction(self.turnCommandExecute)

    def turnCommandExecute(self):
        # Update the odometry that estimates our current position
        robot.drivetrain.updateOdometry()

        # Store the current pose of the robot
        self.currentPose = robot.drivetrain.getSwervePose()
        self.currentAngle = self.currentPose.rotation()

        speed = self.getSpeed()

        targetModuleStates = [
            SwerveModuleState(speed, angle) for angle in self.wheelAngles
        ]

        robot.drivetrain.setModuleStates(targetModuleStates)

    def getSpeed(self):
        speed = -self.pidController.calculate(self.getAngleOffset()) * self.speedLimit

        if abs(speed) > self.speedLimit:
            return math.copysign(self.speedLimit, speed)

        return speed

    def isFinished(self):
        return abs(self.getAngleOffset()) < self.tolerance

    def getCurrentAngle(self):
        return self.currentAngle.degrees()

    def getAngleOffset(self):
        return robot.drivetrain.getAngleRelativeTo(
            self.desiredAngle, self.getCurrentAngle()
        )

    def end(self, interrupted):
        robot.drivetrain.stop()
        robot.drivetrain.setModuleProfiles(0, turn=False)

        robot.drivetrain.removeAutoPeriodicFunction(self.turnCommandExecute)
