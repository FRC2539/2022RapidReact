from distutils import dist
from commands.shooter.baseshootcommand import BaseShootCommand

import robot

from wpimath.filter import MedianFilter


class HighGoalShootCommand(BaseShootCommand):
    def __init__(self):
        super().__init__()

        self.rpm1Multiplier = 210
        self.rpm2Multiplier = 210

        self.rpm1Base = 2300
        self.rpm2Base = 900

        self.hoodMultiplier = 5

        self.stage2Start = 1  # start at 1 "meter"

        self.rpm1Mult2 = 380
        self.rpm2Mult2 = 480
        self.hoodMult2 = 5.5

        self.hoodBase = robot.hood.highGoalAngle

        # Create a filter to improve consistency in distance readings
        self.distanceFilter = MedianFilter(3)

    def initialize(self):
        self.distanceFilter.reset()
        self.configureLimelightPipeline()
        # self.setRPMs(0, 0)
        # self.setHoodPosition(0)

    def execute(self):
        distance = self.distanceFilter.calculate(robot.limelight.calculateDistance())

        # Calculate new rpm and hood values based on the current distance
        self.updateRPMs(distance)
        self.updateHoodAngle(distance)

        # Run core class methods
        self.shootIfShooterAtSpeed()
        self.updateHoodPosition()

    def updateRPMs(self, distance):
        if distance < self.stage2Start:
            rpm1 = self.rpm1Multiplier * distance + self.rpm1Base
            rpm2 = self.rpm2Multiplier * distance + self.rpm2Base
        else:
            rpm1 = self.rpm1Mult2 * distance + self.rpm1Base
            rpm2 = self.rpm2Mult2 * distance + self.rpm2Base

        self.setRPMs(rpm1, rpm2)

    def updateHoodAngle(self, distance):
        if distance < self.stage2Start:
            hoodAngle = self.hoodMultiplier * distance + self.hoodBase
        else:
            hoodAngle = self.hoodMult2 * distance + self.hoodBase

        self.setHoodPosition(hoodAngle)
