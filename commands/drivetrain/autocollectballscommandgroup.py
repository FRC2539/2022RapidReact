from commands2 import ParallelRaceGroup

from commands.drivetrain.autocollectballscommand import AutoCollectBallsCommand

from commands.intake.intakecommand import IntakeCommand
from commands.ballsystem.forwardballsystemcommand import ForwardBallSystemCommand


class AutoCollectBallsCommandGroup(ParallelRaceGroup):
    def __init__(self, endOnBallPickup=True, pickupTwo=True):
        super().__init__()

        self.addCommands(
            IntakeCommand(),
            ForwardBallSystemCommand(useLights=False),
            AutoCollectBallsCommand(
                endOnBallPickup=endOnBallPickup, pickupTwo=pickupTwo
            ),
        )
