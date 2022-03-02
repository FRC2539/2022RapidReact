from commands2 import CommandBase

import robot

import math

from wpimath.geometry import Transform2d, Pose2d, Rotation2d
from wpimath.kinematics import ChassisSpeeds, SwerveModuleState
from wpilib import Timer


class MoveForwardCommand(CommandBase):
    """
    Uses a non-wpilib algorithm to move to position relative to the robot.
    """

    def __init__(
        self,
        distance=0,
        maxSpeed=0.5,
        minSpeed=0.05,
        distanceTolerance=0.15,
        slowdownDistance=0.5,
    ):
        super().__init__()

        self.addRequirements(robot.drivetrain)

        # Store the arguments
        self.maxSpeed = maxSpeed
        self.minSpeed = minSpeed
        self.distanceTolerance = distanceTolerance
        self.slowdownDistance = slowdownDistance
        self.targetDistance = -distance

        self.wheelAngleTolerance = 1.85  # degrees

        self.emergencyStopTimer = Timer()

        self.targetTime = abs(self.targetDistance) / self.maxSpeed + 0.5

    def initialize(self):
        self.emergencyStopTimer.reset()
        self.emergencyStopTimer.start()

        print(self.getRobotPose())

        # Store the initial location of the robot
        self.initialPosition = self.getRobotPose()

        # Calculate the final location of the robot
        # relative to the current location
        self.finalPosition = self.calculateFinalRobotPose()

        print(f"{self.finalPosition=}")

        self.moduleStatesMatch = False

        # Calculate the wheel angles needed
        robot.drivetrain.setModuleStates((SwerveModuleState(0, Rotation2d(0)),) * 4)

        self.enteredSlowdownRange = False

    def execute(self):
        # Wait for the wheel angles to face the correct direction
        if not self.moduleStatesMatch and not self.moduleStateAnglesMatch():
            return

        # Set the chassis speeds for the robot
        robot.drivetrain.setChassisSpeeds(ChassisSpeeds(self.calculateSpeed(), 0, 0))

    def isFinished(self):
        # Returns whether or not the robot has reached the target (distance based)
        if self.calculateDistanceToFinalPose() <= self.distanceTolerance:
            return True
        elif (
            self.calculateDistanceToFinalPose() > self.slowdownDistance
            and self.enteredSlowdownRange
        ):
            return True

        if self.emergencyStopTimer.hasElapsed(self.targetTime):
            return True

        return False

    def end(self, interrupted):
        robot.drivetrain.stop()

    def calculateSpeed(self):
        distance = self.calculateDistanceToFinalPose()

        speed = 0

        # Chooses a speed based on the distance to the target pose
        if distance <= self.slowdownDistance:
            speed = self.minSpeed
            self.enteredSlowdownRange = True
        else:
            speed = self.maxSpeed

        speed = math.copysign(speed, self.targetDistance)

        # Return field relative speeds in the correct direction
        return speed

    def moduleStateAnglesMatch(self):
        """checks if all of the wheels are facing forwards as per forwards move command"""
        currentModuleStates = robot.drivetrain.getModuleStates()
        targetModuleStates = (SwerveModuleState(0, Rotation2d(0)),) * 4
        match = True

        # Check if any of the modules are not facing the correct angle
        for i in range(0, len(targetModuleStates)):
            currentAngle = currentModuleStates[i].angle.degrees()
            targetAngle = targetModuleStates[i].angle.degrees()

            angleDistance = abs(currentAngle - targetAngle)

            # Check if the wheel is at the correct angle
            if not (
                angleDistance <= self.wheelAngleTolerance
                or angleDistance >= (360 - self.wheelAngleTolerance)
            ):  # degree tolerance
                match = False

        # Update the state variable
        self.moduleStatesMatch = match

        return match

    def calculateDistanceToFinalPose(self):
        distance = (
            self.getRobotPose().translation().distance(self.finalPosition.translation())
        )

        print(f"{distance=}")

        return distance

    def calculateFinalRobotPose(self):

        distance = self.targetDistance

        # Calculate either the absolute or relative target position
        x_del, y_del = distance * math.cos(self.getRobotAngle()), distance * math.sin(
            self.getRobotAngle()
        )

        finalPose = Pose2d(
            self.initialPosition.X() + x_del,
            self.initialPosition.Y() + y_del,
            self.getRobotAngle(),
        )

        print(f"{finalPose=}")

        return finalPose

    def getRobotPose(self):
        pose = robot.drivetrain.getSwervePose()
        return pose

    def getRobotAngle(self):
        return self.getRobotPose().rotation().radians()
