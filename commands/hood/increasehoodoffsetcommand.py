from commands2 import InstantCommand

import robot


class IncreaseHoodOffsetCommand(InstantCommand):
    def __init__(self):
        super().__init__()

    def initialize(self):
        robot.hood.increaseHoodOffset()
