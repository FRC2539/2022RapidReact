from commands.shooter.baseshootcommand import BaseShootCommand

import robot


class HighGoalFenderCommand(BaseShootCommand):
    def __init__(self):
        super().__init__()

    def initialize(self):
        self.setRPMs(robot.shooter.highGoalRPM1, robot.shooter.highGoalRPM2)
        self.setCloseHoodPosition()
        self.resetShooterAtRPM()

    def execute(self):
        self.shootIfShooterAtSpeed()
