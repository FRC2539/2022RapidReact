from commands2 import SequentialCommandGroup

from commands.drivetrain.zerogyrocommand import ZeroGyroCommand
from commands.drivetrain.resetposeestimatecommand import ResetPoseEstimateCommand

from wpimath.geometry import Pose2d, Translation2d, Rotation2d


class ResetAutoStateCommand(SequentialCommandGroup):
    def __init__(self, x=0, y=0, angle=0):
        super().__init__()

        # Convert the parameters to a Pose2d object
        initialPose = Pose2d(Translation2d(x, y), Rotation2d.fromDegrees(angle))

        self.addCommands(ZeroGyroCommand(angle), ResetPoseEstimateCommand(initialPose))
