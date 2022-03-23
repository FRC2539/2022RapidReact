from commands.shooter.baseshootcommand import BaseShootCommand

import robot


class SetInitialHoodAngleCommand(BaseShootCommand):
    def __init__(self):
        super().__init__()

    def initialize(self):
        # self.setHoodPosition(robot.shooter.behindLineAngle)
        # self.setRPMs(robot.shooter.behindLineRPM1, robot.shooter.behindLineRPM2)
        self.setHoodPosition(23)
        self.setRPMs(robot.shooter.startRPM1, robot.shooter.startRPM2)

    def execute(self):
        self.updateHoodPosition()
