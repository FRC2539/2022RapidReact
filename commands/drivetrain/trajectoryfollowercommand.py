from commands2 import CommandBase

import robot

from wpimath.geometry import Translation2d, Rotation2d, Pose2d

import constants

from wpilib import Timer


class TrajectoryFollowerCommand(CommandBase):
    """
    Follows a wpilib trajectory. Meant to be a replacement for the SwerveControllerCommand.
    """

    def __init__(self, trajectory):
        super().__init__()

        self.addRequirements(robot.drivetrain)

        self.trajectory = trajectory

        # Store useful data about the trajectory
        self.trajectoryDuration = self.trajectory.totalTime()
        self.trajectoryStates = trajectory.states()
        self.desiredHeading = self.trajectoryStates[len(self.trajectoryStates) - 1].pose.rotation()

        self.driveController = robot.drivetrain.driveController

        self.timer = Timer()

        # Useful for position based validation (not currently implemented)
        self.tolerance = constants.drivetrain.autoTolerance

    def initialize(self):
        # Start a timer
        self.timer.reset()
        self.timer.start()

    def execute(self):
        # Update the pose estimator
        robot.drivetrain.updateOdometry()

        # Get the current estimated robot pose
        currentPose = robot.drivetrain.getSwervePose()

        # Get the current desired state from the trajectory
        desiredState = self.trajectory.sample(self.timer.get())

        # Calculate the required chassis speeds for getting to the desired location
        chassisSpeeds = self.driveController.calculate(
            currentPose, desiredState, self.desiredHeading
        )

        # Have the swerve drivetrain follow the required speeds
        robot.drivetrain.setChassisSpeeds(chassisSpeeds)

    def isFinished(self):
        # End the command if the duration of the trajectory has elapsed
        return self.timer.hasElapsed(self.trajectoryDuration)

    def end(self, interrupted):
        # Stop the timer
        self.timer.stop()

        # Stop the robot
        robot.drivetrain.stop()