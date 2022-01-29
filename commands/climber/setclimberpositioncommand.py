from commands2 import CommandBase

import robot


class SetClimberPositionCommand(CommandBase):
    def __init__(self, position):
        super().__init__()

        self.addRequirements(robot.climber)

        self.position = position

        self.isValidPosition = robot.climber.isValidClimberPosition(position)

        # Determine if the climber needs to be raised or lowered
        self.raiseClimber = (position - robot.climber.getPosition()) > 0

        # Determine which climbing method we need to use
        self.moveClimber = (
            robot.climber.raiseClimber
            if self.raiseClimber
            else robot.climber.lowerClimber
        )

        # Determine which comparison method to use
        self.climberInPosition = (
            robot.climber.abovePosition
            if self.raiseClimber
            else robot.climber.belowPosition
        )

    def execute(self):
        self.moveClimber()

    def isFinished(self):
        return not self.isValidPosition or self.climberInPosition(self.position)

    def end(self, interrupted):
        robot.climber.stopClimber()
