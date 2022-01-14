from commands2 import InstantCommand

import robot


class ToggleFieldOrientationCommand(InstantCommand):

    """
    Toggles the field orientation of the 2021 Swerve robot.
    Field-centric, the default setting, maintains a constant
    sense of "forward". To a robot-centric control standpoint,
    forward is whatever way the robot is facing. Think of a
    tank drivetrain; this is a robot-centric drivetrain.
    """

    def __init__(self, set_=None):
        super().__init__()

        self.addRequirements(robot.drivetrain)

        self.set_ = set_

    def initialize(self):
        if self.set_ is None:
            robot.drivetrain.isFieldOriented = not robot.drivetrain.isFieldOriented
        else:
            robot.drivetrain.isFieldOriented = self.set_
