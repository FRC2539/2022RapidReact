from commands2 import ParallelCommandGroup

from commands.drivetrain.autocollectballscommand import AutoCollectBallsCommand

from commands.intake.intakecommand import IntakeCommand
from commands.ballsystem.forwardballsystemcommand import ForwardBallSystemCommand


class AutoCollectBallsCommandGroup(ParallelCommandGroup):
    def __init__(self):
        super().__init__()

        self.addCommands(
            IntakeCommand(), ForwardBallSystemCommand(), AutoCollectBallsCommand()
        )
