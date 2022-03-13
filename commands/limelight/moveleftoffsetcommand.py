from commands2 import InstantCommand

import robot
import constants


class MoveLeftOffsetCommand(InstantCommand):
    def __init__(self):
        super().__init__()

    def initialize(self):
        constants.limelight.xOffset += constants.limelight.xOffsetStep
        robot.limelight.updateXOffset()
