from commands2 import InstantCommand

import robot

from wpimath.geometry import Pose2d


class ResetPoseEstimateCommand(InstantCommand):
    def __init__(self, initialPose=Pose2d()):
        super().__init__()

        self.addRequirements(robot.drivetrain)

        self.initialPose = initialPose

    def initialize(self):
        robot.drivetrain.resetPoseEstimate(pose=self.initialPose)
