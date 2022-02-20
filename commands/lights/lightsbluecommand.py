from commands2 import CommandBase

import robot


class LightsBlueCommand(CommandBase):
    def __init__(self):
        super().__init__()

        self.addRequirements(robot.lights)

    def initialize(self):
        robot.lights.solidBlue()

    def execute(self):
        robot.lights.solidBlue()

    def end(self):
        robot.lights.off()
