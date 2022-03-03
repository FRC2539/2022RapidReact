from commands.shooter.baseshootcommand import BaseShootCommand

import robot


class CustomShootCommand(BaseShootCommand):
    def __init__(self, rpm1=None, rpm2=None, hoodAngle=None):
        super().__init__()

        self.rpm1 = rpm1
        self.rpm2 = rpm2
        self.hoodAngle = hoodAngle

    def initialize(self):
        rpm1 = self.rpm1 if self.rpm1 is not None else robot.shooter.testRPM1
        rpm2 = self.rpm2 if self.rpm2 is not None else robot.shooter.testRPM2
        angle = (
            self.hoodAngle if self.hoodAngle is not None else robot.hood.highGoalAngle
        )

        self.setRPMs(rpm1, rpm2)
        self.setHoodPosition(angle)

    def execute(self):
        self.shootIfShooterAtSpeed()
        self.updateHoodPosition()
