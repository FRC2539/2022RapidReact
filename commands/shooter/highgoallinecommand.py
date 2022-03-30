from commands.shooter.baseshootcommand import BaseShootCommand

import robot


class HighGoalLineCommand(BaseShootCommand):
    def __init__(self):
        super().__init__()

    def initialize(self):
        self.setRPMs(robot.shooter.behindLineRPM1, robot.shooter.behindLineRPM2)
        self.setFarHoodPosition()
        self.resetShooterAtRPM()

    def execute(self):
        self.shootIfShooterAtSpeed()
