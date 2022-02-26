from commands2 import CommandBase

import robot


class ManualSetHoodPositionCommand(CommandBase):
    def __init__(self, position):
        super().__init__()

        self.addRequirements(robot.hood)

        self.position = position

        self.tolerance = 0.5

    def execute(self):
        distance = abs(self.position - robot.hood.getPosition())
        speed = 0.1 if distance > 2 else 0.03

        direction = 1 if (self.position - robot.hood.getPosition()) >= 0 else -1
        robot.hood.move(speed * direction)

    def isFinished(self):
        return abs(self.position - robot.hood.getPosition()) <= self.tolerance

    def end(self, interrupted):
        robot.hood.stop()
