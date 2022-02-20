from commands2 import CommandBase

import robot


class LightsOrangeCommand(CommandBase):
    def __init__(self):
        super().__init__()

        self.addRequirements(robot.lights)

    def initialize(self):
        robot.lights.solidOrange()

    def execute(self):
        robot.lights.solidOrange()

    def end(self):
        robot.lights.off()
