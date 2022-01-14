from commands2 import CommandBase
from wpimath.geometry._geometry import Pose2d
from wpimath.kinematics._kinematics import ChassisSpeeds
import robot

from wpimath.geometry import Rotation2d, Translation2d, Transform2d

from wpimath.kinematics import SwerveModuleState

import constants


class MoveCommand(CommandBase):
    def __init__(self, x, y, speed=0.005, tolerance=1):
        """
        Moves the robot to a position x meters forward/backward
        and y meters left/right.
        """

        # TODO either generate an acceleration curve, or, more preferably,
        # use trajectories

        # Update, use intermediate points

        super().__init__()

        # Store where the robot should move to
        self.targetPosition = Translation2d(x, y)

        self.tolerance = tolerance

        self.driveController = robot.drivetrain.driveController

        # Make the speed a percent of the robot's speed limit
        self.speed = speed * constants.drivetrain.speedLimit

        self.addRequirements(robot.drivetrain)

    def initialize(self):
        self.transformToPosition = Transform2d(self.targetPosition, Rotation2d(0))

        robot.drivetrain.setModuleProfiles(1, turn=False)

        # Get the current state of the robot
        self.currentPose = robot.drivetrain.getSwervePose()

        # Convert the position we are given to a pose
        self.desiredPose = self.currentPose.transformBy(self.transformToPosition)
        # self.desiredPose = Pose2d(self.targetPosition, Rotation2d(0))

        robot.drivetrain.addAutoPeriodicFunction(self.moveCommandExecute)

    def moveCommandExecute(self):
        # Update the odometry that estimates our current position
        robot.drivetrain.updateOdometry()

        # Get the current state of the robot
        self.currentPose = robot.drivetrain.getSwervePose()

        chassisSpeeds = self.driveController.calculate(
            self.currentPose, self.desiredPose, self.speed, Rotation2d(0)
        )

        robot.drivetrain.setChassisSpeeds(chassisSpeeds)

    def isFinished(self):
        return (
            self.currentPose.translation().distance(self.desiredPose.translation())
            < self.tolerance
        )

    def end(self, interrupted):
        robot.drivetrain.stop()
        robot.drivetrain.setModuleProfiles(0, turn=False)

        robot.drivetrain.removeAutoPeriodicFunction(self.moveCommandExecute)
