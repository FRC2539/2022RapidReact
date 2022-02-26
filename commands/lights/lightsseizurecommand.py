from commands2 import CommandBase

import robot

import time


class LightsSeizureCommand(CommandBase):
    def __init__(self):
        super().__init__()

        self.addRequirements(robot.lights)

        # self.setRunWhenDisabled(True)

        self.colorvalue = 0
        # NetworkTables.initialize(server='roborio-2539-frc.local')

    def initialize(self):
        pass

    def execute(self):
        if self.colorvalue <= 1:
            robot.lights.solidRed()
            self.colorvalue += 1

        elif self.colorvalue <= 2:
            robot.lights.solidGreen()
            self.colorvalue += 1

        elif self.colorvalue <= 3:
            robot.lights.solidBlue()
            self.colorvalue = 1

        # time.sleep(0.02)

    def end(self, interrupted):
        robot.lights.off()
