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
        self.tolerance = constants.drivetrain.autoTolerance

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

        print(self.poses)
        print(self.currentPose)

        self.driveController.setTolerance(self.tolerance)

        # Store the initial pose of the robot
        self.initialPose = self.getRobotPose()

        self.calculateDesiredAbsolutePose()

        # print(self.poses)

        # Start the wheels facing in the correct direction
        self.zeroSpeedModuleStates = self.calculateInitialModuleStates()

        robot.drivetrain.setModuleStates(self.zeroSpeedModuleStates)

    def execute(self):
        if not self.moduleStateAnglesMatch(self.zeroSpeedModuleStates):
            pass

        # Calculate the chassis speeds (x', y', omega) to reach the desired pose
        chassisSpeeds = self.calculateChassisSpeeds()
        # chassisSpeeds = ChassisSpeeds(self.linearVelocity)

        # print(chassisSpeeds)
        # print(self.getRobotPose())

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

    def calculateChassisSpeeds(self):
        return self.driveController.calculate(
            self.getRobotPose(), self.desiredPose, self.linearVelocity, self.angleRef
        )

        # Convert to robot relative speeds
        # return robot.drivetrain.getRobotRelativeSpeedsFromFieldSpeeds(
        #     chassisSpeeds.vx, chassisSpeeds.vy, chassisSpeeds.omega
        # )

    def moduleStateAnglesMatch(self, targetModuleStates):
        currentModuleStates = robot.drivetrain.getModuleStates()

        match = True

        for i in range(0, len(targetModuleStates)):
            currentAngle = currentModuleStates[i].angle.degrees()
            targetAngle = targetModuleStates[i].angle.degrees()

            if abs(currentAngle - targetAngle) > 2:  # 5 degree tolerance
                match = False

        return match

    def calculateInitialModuleStates(self):
        chassisSpeeds = self.calculateChassisSpeeds()

        print("chassis speeds")
        print(chassisSpeeds)

        moduleStates = robot.drivetrain.convertChassisSpeedsToModuleStates(
            chassisSpeeds
        )

        optimizedModuleStates = robot.drivetrain.optimizeModuleStates(moduleStates)

        # Set the module states to their correct angles
        for moduleState in optimizedModuleStates:
            moduleState.speed = 0

        return optimizedModuleStates

    def getRobotPose(self):
        return robot.drivetrain.getSwervePose()

    def getDesiredRelativePose(self):
        return self.poses[self.currentPose]

    def resetCurrentPose(self):
        self.currentPose = 0

    def incrementPose(self):
        self.currentPose += 1
        print(self.currentPose)

    def calculateDesiredAbsolutePose(self):
        # Get the desired pose,
        # and make it relative to the starting location of the robot
        self.desiredPose = self.getDesiredRelativePose().relativeTo(self.initialPose)

        # Either face forward or follow the heading of the desired pose
        self.angleRef = (
            self.desiredPose.rotation() if self.matchHeading else Rotation2d(0)
        )

        print(self.desiredPose)

    def isLastPose(self):
        return self.currentPose == len(self.poses) - 1
