from commands2 import SequentialCommandGroup, WaitCommand

import robot

from commands.climber.setclimberpositioncommand import SetClimberPositionCommand
from commands.climber.extendclimberarmcommand import ExtendClimberArmCommand
from commands.climber.retractclimberarmcommand import RetractClimberArmCommand

import constants


class ClimbBarCommand(SequentialCommandGroup):
    """
    A repeatable command for climbing from one bar to the next.

    It first raises the arm all the way up. Next,
    it raises the arm to the perpendicular position.
    Then, wait a little, before pulling all the way down to get the static hooks on.
    Finally, raise the climber arm position a bit, and move it to the angled position.
    """

    def __init__(self):
        super().__init__()

        self.addRequirements(robot.climber)

        self.addCommands(
            SetClimberPositionCommand(constants.climber.upperLimit),
            ExtendClimberArmCommand(),
            WaitCommand(0.4),
            SetClimberPositionCommand(constants.climber.lowerLimit),
            SetClimberPositionCommand(constants.climber.lowerLimit + 30000),
            RetractClimberArmCommand(),
        )
