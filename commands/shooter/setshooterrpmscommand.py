from commands2 import CommandBase

import robot


class SetShooterRPMsCommand(CommandBase):
    def __init__(self, rpm1, rpm2):
        super().__init__()

        self.rpm1 = rpm1
        self.rpm2 = rpm2

        self.addRequirements(robot.shooter)

    def initialize(self):
        # robot.shooter.setRPM(self.rpm1, self.rpm2)
        robot.shooter.setPercent(0.4, 0.4)

    def end(self, interrupted):
        robot.shooter.stopShooter()
