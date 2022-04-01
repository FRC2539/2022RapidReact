from commands.shooter.baseshootcommand import BaseShootCommand

import robot


class SetInitialHoodAngleCommand(BaseShootCommand):
    def __init__(self):
        super().__init__()

    def initialize(self):
        self.setRPMs(robot.shooter.startRPM1, robot.shooter.startRPM2)
        self.setFarHoodPosition()
