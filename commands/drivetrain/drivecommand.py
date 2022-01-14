from commands2 import CommandBase

import constants
import robot
from controller import logicalaxes
from custom import driverhud
import math


logicalaxes.registerAxis("forward")
logicalaxes.registerAxis("strafe")
logicalaxes.registerAxis("rotate")


class DriveCommand(CommandBase):
    def __init__(self):
        super().__init__()

        self.addRequirements(robot.drivetrain)

        robot.drivetrain.resetGyro()
        robot.drivetrain.resetOdometry()

    def initialize(self):
        robot.drivetrain.stop()
        robot.drivetrain.setProfile(0)
        robot.drivetrain.resetEncoders()

        self.lastX = None
        self.slowed = False

    def execute(self):
        # Avoid quick changes in direction
        x = logicalaxes.forward.get()
        if self.lastX is None:
            self.lastX = x
        else:
            cooldown = 0.05
            self.lastX -= math.copysign(cooldown, self.lastX)

            # If the sign has changed, don't move
            if self.lastX * x < 0:
                x = 0

            if abs(x) > abs(self.lastX):
                self.lastX = x

        # x - forward and backward axis (these axes are relative to the field)
        # y - side to side axis
        # rotate - counterclockwise rotation is positive

        robot.drivetrain.move(x, logicalaxes.strafe.get(), logicalaxes.rotate.get())
