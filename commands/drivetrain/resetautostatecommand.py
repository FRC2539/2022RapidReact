from commands2 import SequentialCommandGroup

from commands.drivetrain.zerogyrocommand import ZeroGyroCommand
from commands.drivetrain.resetposeestimatecommand import ResetPoseEstimateCommand


class ResetAutoStateCommand(SequentialCommandGroup):
    def __init__(self, angle=0):
        super().__init__()

        self.addCommands(ZeroGyroCommand(angle), ResetPoseEstimateCommand())
