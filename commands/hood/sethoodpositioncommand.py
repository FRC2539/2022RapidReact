from commands2 import InstantCommand

import robot


class SetHoodPositionCommand(InstantCommand):
    """
    Use the PID controller to move the hood to a specific location.

    Note: the subsystem automatically checks to make the sure location is valid
    """

    def __init__(self, position):
        super().__init__()

        self.addRequirements(robot.hood)

        self.position = position

    def initialize(self):
        robot.hood.setPosition(self.position)
