from commands.shooter.baseshootcommand import BaseShootCommand

import robot


class HighFenderSpinupCommand(BaseShootCommand):
    def __init__(self):
        super().__init__()

    def initialize(self):
        self.setRPMs(robot.shooter.highGoalRPM1, robot.shooter.highGoalRPM2)
        self.setHoodPosition(robot.hood.highGoalAngle)
        self.resetShooterAtRPM()

    def execute(self):
        self.updateHoodPosition()
