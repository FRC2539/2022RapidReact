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
        if robot.climber.isSolenoidOff():
            robot.climber.retractClimberArm()
        else:
            self.toggleSolenoid()

    def toggleSolenoid(self):
        if robot.climber.isSolenoidForward():
            robot.climber.retractClimberArm()
        else:
            robot.climber.extendClimberArm()
