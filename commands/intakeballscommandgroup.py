from commands2 import ParallelCommandGroup

from commands.intake.intakecommand import IntakeCommand

# from commands.ballsystem.forwardconveyorcommand import ForwardConveyorCommand
# from commands.ballsystem.runchamberuntilballpresentcommand import (
#     RunChamberUntilBallPresentCommand,
# )
from commands.ballsystem.forwardballsystemcommand import ForwardBallSystemCommand


class IntakeBallsCommandGroup(ParallelCommandGroup):
    def __init__(self):
        super().__init__()

        self.addCommands(
            IntakeCommand(),
            ForwardBallSystemCommand(),
        )
