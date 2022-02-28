from commands2 import InstantCommand

import robot


class ToggleClimberSolenoidCommand(InstantCommand):
    """
    Toggles the state of the climber solenoid (piston).
    """

    def __init__(self):
        super().__init__()

        self.addRequirements(robot.climber)

    def initialize(self):
        # Activate the solenoid if it is currently off,
        # or toggle the current state of the solenoid
        if robot.pneumatics.isSolenoidOff():
            robot.pneumatics.retractClimberArm()
        else:
            self.toggleSolenoid()

    def toggleSolenoid(self):
        if robot.pneumatics.isSolenoidForward():
            robot.pneumatics.retractClimberArm()
        else:
            robot.pneumatics.extendClimberArm()
