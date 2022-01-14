from commands2 import CommandBase

import robot

from wpimath.geometry import Translation2d, Rotation2d, Pose2d

from wpimath.kinematics import ChassisSpeeds

import constants

from wpilib import Timer


class TrajectoryFollowerCommand(CommandBase):
    def __init__(self, trajectory, distanceTolerance=0.1):
        super().__init__()

        self.addRequirements(robot.drivetrain)

        self.trajectory = trajectory

        self.driveController = robot.drivetrain.driveController

        self.timer = Timer()

        self.distanceTolerance = distanceTolerance

    def initialize(self):
        # robot.drivetrain.setModuleProfiles(1, turn=False)

        self.timer.reset()

        self.timer.start()

        self.trajectoryDuration = self.trajectory.totalTime()

        self.finalState = self.trajectory.sample(self.trajectoryDuration).pose

        robot.drivetrain.addAutoPeriodicFunction(self.trajectoryFollowerExecute)

    def trajectoryFollowerExecute(self):
        robot.drivetrain.updateOdometry()

        self.currentPose = robot.drivetrain.getSwervePose()

        self.currentTime = self.timer.get()

        trajectoryState = self.trajectory.sample(
            self.currentTime + constants.drivetrain.autoPeriodicPeriod
        )

        # trajectoryPose = trajectoryState.pose

        # linearVelocityRef = constants.drivetrain.autoSpeedLimit

        # heading = trajectoryState.pose.rotation()
        heading = Rotation2d(0)

        chassisSpeeds = self.driveController.calculate(
            self.currentPose, trajectoryState, heading
        )
        # chassisSpeeds = self.driveController.calculate(self.currentPose, trajectoryPose, linearVelocityRef, heading)

        robot.drivetrain.setChassisSpeeds(chassisSpeeds)

    def isFinished(self):
        self.currentTime = self.timer.get()

        timeUp = self.currentTime >= self.trajectoryDuration

        self.currentPose = robot.drivetrain.getSwervePose()

        distanceToTargetPosition = self.currentPose.translation().distance(
            self.finalState.translation()
        )

        atPosition = distanceToTargetPosition <= self.distanceTolerance

        return timeUp or atPosition

    def end(self, interrupted):
        self.timer.stop()

        robot.drivetrain.removeAutoPeriodicFunction(self.trajectoryFollowerExecute)

        # robot.drivetrain.setModuleProfiles(0, turn=False)
