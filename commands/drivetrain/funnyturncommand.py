from commands2 import CommandBase

import robot
import constants

import math


class FunnyTurnCommand(CommandBase):
    """Allows autonomous turning using the drive base encoders."""

    def __init__(self, degrees, tolerance=3):
        super().__init__()

        self.degrees = degrees
        self.tolerance = tolerance
        # Radius (in) * 2 * pi
        self.robotCircumference = constants.drivetrain.robotRadius * math.pi * 2

        self.addRequirements(robot.drivetrain)

    def calculateDisplacement(self):
        """Returns the distance (in) for the given degrees.
        This feeds into the drivetrain's positioning method,
        where the distance is based on the robot's circumference."""
        # Angle -> percentage of the robot's circumference
        return (self.degrees / 360) * self.robotCircumference

    def initialize(self):
        """Calculates new positions by offseting the current ones."""

        robot.drivetrain.setModuleProfiles(1, turn=False)

        self.modulesInPosition = False
        self.turnSet = False

        self.targetAngles = [135, 45, 225, 315]
        self.startAngle = robot.drivetrain.getAngle()

        # Rotate the swerve modules to a position where they can rotate in a circle.
        robot.drivetrain.setModuleAngles(self.targetAngles)

        self.targetDistance = self.calculateDisplacement()

        # 316, 227

    def execute(self):
        print("runnning\n\n\n\n\n\n\n\n\n\n\n\n")
        if self.modulesInPosition and not self.turnSet:
            robot.drivetrain.setPositions(
                [
                    self.targetDistance,
                    self.targetDistance,
                    self.targetDistance,
                    self.targetDistance,
                ]
            )
            self.turnSet = True

        if not self.modulesInPosition:
            count = 0
            for angle, desiredAngle in zip(
                robot.drivetrain.getModuleAngles(), [135, 45, 225, 315]
            ):
                if abs(angle - desiredAngle) < self.tolerance:
                    count += 1

            if count == 4:
                self.modulesInPosition = True

    def isFinished(self):
        return abs(robot.drivetrain.getAngleTo(self.startAngle)) + self.tolerance > abs(
            self.degrees
        )

    def end(self, interrupted):
        robot.drivetrain.stop()
        robot.drivetrain.setModuleProfiles(0, turn=False)
