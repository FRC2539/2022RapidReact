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

        # Use the drive theta controller from the drivetrain subsystem
        self.pidController = robot.drivetrain.driveThetaController

    def initialize(self):
        robot.drivetrain.stop()
        robot.drivetrain.setProfile(0)

        self.lastX = None
        self.slowed = False

        robot.limelight.setPipeline(1)

    def execute(self):
        # TODO replace the interesting code below with a rate limiter object
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

        # Determine the rotation value (-1 to 1)
        # Note: the method used depends on if the limelight lock is enabled
        # rotate = (

        #     if robot.drivetrain.isLimelightLockEnabled()
        #     else
        # )

        # x - forward and backward axis (these axes are relative to the field)
        # y - side to side axis
        # rotate - counterclockwise rotation is positive

        if robot.drivetrain.isLimelightLockEnabled():
            robot.drivetrain.move(
                x,
                logicalaxes.strafe.get(),
                robot.limelight.calculateTurnPercent(),  # robot.limelight.calculateTurnVelocity()
            )  # robot.limelight.calculateTurnPercent())
            if robot.limelight.isAimed():
                robot.lights.solidGreen()
            else:
                robot.lights.showTeamColor()
        else:
            robot.drivetrain.move(x, logicalaxes.strafe.get(), logicalaxes.rotate.get())
