from commands2 import CommandBase

import robot


class IntakeCommand(CommandBase):
    def __init__(self):
        super().__init__()

        self.addRequirements(robot.intake)
        self.addRequirements(robot.pneumatics)

    def initialize(self):
        robot.pneumatics.extendIntake()
        robot.intake.intakeBalls()

    def end(self, interrupted):
        robot.intake.stop()
        robot.pneumatics.retractIntake()
