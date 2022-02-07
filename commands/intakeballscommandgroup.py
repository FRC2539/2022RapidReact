from commands2 import ParallelCommandGroup

import robot

from commands.intake.intakecommand import IntakeCommand
from commands.ballsystem.forwardconveyorcommand import ForwardConveyorCommand


class IntakeBallsCommandGroup(ParallelCommandGroup):
    def __init__(self):
        super().__init__()

        self.addCommands(IntakeCommand(), ForwardConveyorCommand())
