from commands2 import CommandBase

import robot


class ToggleClimberSolenoidCommand(CommandBase):
    """
    Toggles the state of the climber solenoid (piston).
    """

    def __init__(self):
        super().__init__()

        self.addRequirements(robot.climber)

    def initialize(self):
        # Activate the solenoid if it is currently off,
        # or toggle the current state of the solenoid
        if robot.climber.isSolenoidOff():
            robot.climber.extendClimberArm()
        else:
            robot.climber.toggleClimberArm()
