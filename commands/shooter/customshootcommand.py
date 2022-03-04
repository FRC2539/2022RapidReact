from commands.shooter.baseshootcommand import BaseShootCommand

import robot


class CustomShootCommand(BaseShootCommand):
    def __init__(self, rpm1=None, rpm2=None, hoodAngle=None):
        super().__init__()

        self.customrpm1 = rpm1
        self.customrpm2 = rpm2
        self.angle = hoodAngle

    def initialize(self):
        rpm1 = (
            self.customrpm1 if self.customrpm1 is not None else robot.shooter.testRPM1
        )
        rpm2 = (
            self.customrpm2 if self.customrpm2 is not None else robot.shooter.testRPM2
        )
        angle = self.angle if self.angle is not None else robot.hood.testAngle

        self.setRPMs(rpm1, rpm2)
        self.setHoodPosition(angle)

    def execute(self):
        self.shootIfShooterAtSpeed()
        self.updateHoodPosition()
