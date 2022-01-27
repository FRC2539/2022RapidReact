from commands2 import CommandBase

import robot

from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.kinematics import ChassisSpeeds

import constants


class PointFollowCommand(CommandBase):
    """
    Follows a series of poses (a point and an associated angle).
    Make sure the initial pose is at (0,0) facing 0 degrees (default constructor).

    Can be subclassed to create specific autonomous commands.

    The match heading option determines if the robot faces the direction it is traveling.
    (By default they are uncoupled)
    """

    # TODO: currently overshooting, try decreasing speed and adding more hidden points

    def __init__(self, poses, linearVelocity=None, matchHeading=False):
        super().__init__()

        # Store the given points
        self.poses = poses

        # Store if the robot should match the heading to the direction of travel
        self.matchHeading = matchHeading

        self.addRequirements(robot.drivetrain)

        self.driveController = robot.drivetrain.driveController

        # Set a tolerance of 5 cm, and 5 degrees
        self.tolerance = Pose2d(Translation2d(0.05, 0.05), Rotation2d.fromDegrees(5))

        # Create a location to store the starting pose of the robot
        self.initialPose = Pose2d()

        # Create variables to store data about the current desired pose
        self.desiredPose = Pose2d()
        self.angleRef = Rotation2d()

        # Set the desired linear velocity for the auto
        self.linearVelocity = (
            linearVelocity
            if linearVelocity is not None
            else constants.drivetrain.speedLimit
        )

        # Create a variable to track the current desired pose
        self.currentPose = 0

    def initialize(self):
        self.resetCurrentPose()

        self.driveController.setTolerance(self.tolerance)

        # Store the initial pose of the robot
        self.initialPose = self.getRobotPose()

        self.calculateDesiredAbsolutePose()

        print(self.poses)

    def execute(self):
        # Calculate the chassis speeds (x', y', omega) to reach the desired pose
        chassisSpeeds = self.driveController.calculate(
            self.getRobotPose(), self.desiredPose, self.linearVelocity, self.angleRef
        )

        print(chassisSpeeds)

        # Follow the chassis speeds with the drivetrain
        robot.drivetrain.setChassisSpeeds(chassisSpeeds)

    def isFinished(self):
        # Complete the path following, or go to the next pose in the path
        if self.driveController.atReference():
            if self.isLastPose():
                return True
            else:
                self.incrementPose()
                self.calculateDesiredAbsolutePose()
                return False

    def end(self, interrupted):
        # Stop the robot
        robot.drivetrain.stop()

    def getRobotPose(self):
        return robot.drivetrain.getSwervePose()

    def getDesiredRelativePose(self):
        return self.poses[self.currentPose]

    def resetCurrentPose(self):
        self.currentPose = 0

    def incrementPose(self):
        self.currentPose += 1

    def calculateDesiredAbsolutePose(self):
        # Get the desired pose,
        # and make it relative to the starting location of the robot
        self.desiredPose = self.getDesiredRelativePose().relativeTo(self.initialPose)

        # Either face forward or follow the heading of the desired pose
        self.angleRef = (
            self.desiredPose.rotation() if self.matchHeading else Rotation2d(0)
        )

    def isLastPose(self):
        return self.currentPose == len(self.poses) - 1
