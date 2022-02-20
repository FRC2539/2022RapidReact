from commands2 import ParallelCommandGroup

import robot

from commands.intake.intakecommand import IntakeCommand
from commands.ballsystem.forwardconveyorcommand import ForwardConveyorCommand
from commands.ballsystem.forwardchambercommand import ForwardChamberCommand
from commands.ballsystem.forwardballsystemcommand import ForwardBallSystemCommand
from commands.shooter.setshooterrpmscommand import SetShooterRPMsCommand


class IntakeBallsCommandGroup(ParallelCommandGroup):
    def __init__(self):
        super().__init__()

        self.addCommands(
            IntakeCommand(),
            ForwardBallSystemCommand(),
            SetShooterRPMsCommand(2000, 2000),
        )
