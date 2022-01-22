from commands2 import CommandBase

import robot


class RejectCommand(CommandBase):
    def __init__(self):
        super().__init__()

        self.addRequirements(robot.intake)

    def initialize(self):
        robot.intake.outtakeBalls()

    def end(self, interrupted):
        robot.intake.stop()
