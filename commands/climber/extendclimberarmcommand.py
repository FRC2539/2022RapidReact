from commands2 import InstantCommand

import robot


class ExtendClimberArmCommand(InstantCommand):
    """
    Extends the climber solenoid (piston).
    """

    def __init__(self):
        super().__init__()

        self.addRequirements(robot.climber)

    def initialize(self):
        robot.climber.extendClimberArm()
