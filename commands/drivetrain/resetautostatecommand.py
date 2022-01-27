from commands2 import SequentialCommandGroup

from commands.drivetrain.zerogyrocommand import ZeroGyroCommand
from commands.drivetrain.resetposeestimatecommand import ResetPoseEstimateCommand

from wpimath.geometry import Pose2d


class ResetAutoStateCommand(SequentialCommandGroup):
    def __init__(self, initialPose=Pose2d()):
        super().__init__()

        angle = initialPose.rotation().degrees()

        self.addCommands(ZeroGyroCommand(angle), ResetPoseEstimateCommand(initialPose))
