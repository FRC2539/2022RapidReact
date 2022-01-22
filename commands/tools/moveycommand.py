from commands2 import InstantCommand

import robot


class MoveYCommand(InstantCommand):
    def __init__(self, y):
        super().__init__()

        self.addRequirements(robot.drivetrain)
        self.y = y

    def initialize(self):
        robot.drivetrain.move(0, self.y, 0)
