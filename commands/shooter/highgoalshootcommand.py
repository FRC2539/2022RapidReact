from commands.shooter.baseshootcommand import BaseShootCommand

import robot


class HighGoalShootCommand(BaseShootCommand):
    def __init__(self):
        super().__init__()

        self.rpm1Multiplier = 8
        self.rpm2Multiplier = 8
        self.rpm1Base = robot.shooter.highGoalRPM1
        self.rpm2Base = robot.shooter.highGoalRPM2

        self.hoodMultiplier = 0.1
        self.hoodBase = robot.hood.highGoalAngle

    def initialize(self):
        pass
        # self.setRPMs(0, 0)
        # self.setHoodPosition(0)

    def execute(self):
        distance = robot.limelight.calculateDistance()

        # Calculate new rpm and hood values based on the current distance
        self.updateRPMs(distance)
        self.updateHoodAngle(distance)

        # Run core class methods
        self.shootIfShooterAtSpeed()
        self.updateHoodPosition()

    def updateRPMs(self, distance):
        rpm1 = self.rpm1Multiplier * distance + self.rpm1Base
        rpm2 = self.rpm2Multiplier * distance + self.rpm2Base

        self.setRPMs(rpm1, rpm2)

    def updateHoodAngle(self, distance):
        hoodAngle = self.hoodMultiplier * distance + self.hoodBase

        self.setHoodPosition(hoodAngle)
