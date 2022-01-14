from commands2 import SequentialCommandGroup

from commands.drivetrain.zerogyrocommand import ZeroGyroCommand
from commands.drivetrain.resetodometrycommand import ResetOdometryCommand


class ResetAutoStateCommand(SequentialCommandGroup):
    def __init__(self):
        super().__init__()

        self.addCommands(
            ZeroGyroCommand(driveOrientation=False), ResetOdometryCommand()
        )
