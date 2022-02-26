from commands.shooter.baseshootcommand import BaseShootCommand

import robot


class LowGoalShootCommand(BaseShootCommand):
    def __init__(self):
        super().__init__()

    def initialize(self):
        self.setRPMs(robot.shooter.lowGoalRPM1, robot.shooter.lowGoalRPM2)
        self.setHoodPosition(robot.hood.lowGoalAngle)

    def execute(self):
        self.shootIfShooterAtSpeed()
        self.updateHoodPosition()
