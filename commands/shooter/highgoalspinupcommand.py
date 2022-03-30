from commands.shooter.baseshootcommand import BaseShootCommand
import math

import robot

from wpimath.filter import MedianFilter


class HighGoalSpinupCommand(BaseShootCommand):
    def __init__(self):
        super().__init__()

        # Create a filter to improve consistency in distance readings
        self.distanceFilter = MedianFilter(3)

        self.startDistance = robot.shooter.startDistance  # meters from the target

    def initialize(self):
        self.distanceFilter.reset()
        self.setFarHoodPosition()
        self.resetShooterAtRPM()

    def execute(self):
        distance = (
            self.distanceFilter.calculate(robot.limelight.calculateDistance())
            - self.startDistance
        )

        # Calculate new rpm and hood values based on the current distance
        [rpm1, rpm2] = robot.shooter.calculateRPMsFromDistance(distance)

        self.setRPMs(rpm1, rpm2)
