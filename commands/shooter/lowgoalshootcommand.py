from commands.shooter.baseshootcommand import BaseShootCommand

import robot


class LowGoalShootCommand(BaseShootCommand):
    def __init__(self, rpm1=None, rpm2=None):
        super().__init__()

        self.rpm1 = rpm1 if rpm1 is not None else robot.shooter.lowGoalRPM1
        self.rpm2 = rpm2 if rpm2 is not None else robot.shooter.lowGoalRPM2

    def initialize(self):
        self.setRPMs(self.rpm1, self.rpm2)
        self.setHoodPosition(robot.hood.lowGoalAngle)

    def execute(self):
        self.shootIfShooterAtSpeed()
        self.updateHoodPosition()
