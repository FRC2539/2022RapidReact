from commands2 import InstantCommand

import robot
import constants


class MoveUpOffsetCommand(InstantCommand):
    def __init__(self):
        super().__init__()

    def initialize(self):
        constants.limelight.yOffset += constants.limelight.yOffsetStep
        robot.limelight.updateYOffset()
