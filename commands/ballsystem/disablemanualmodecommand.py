from commands2 import InstantCommand

import robot


class DisableManualModeCommand(InstantCommand):
    def __init__(self):
        super().__init__()

    def initialize(self):
        robot.ballsystem.disableManualBallIntake()
