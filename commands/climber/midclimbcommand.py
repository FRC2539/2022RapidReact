from commands2 import SequentialCommandGroup, WaitCommand

from commands.climber.setclimberpositioncommand import SetClimberPositionCommand

import constants


class MidClimbCommand(SequentialCommandGroup):
    def __init__(self):
        super().__init__()

        self.addCommands(
            SetClimberPositionCommand(constants.climber.upperLimit),
            WaitCommand(1),
            SetClimberPositionCommand(constants.climber.lowerLimit),
        )
