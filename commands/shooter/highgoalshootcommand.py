from commands.shooter.baseshootcommand import BaseShootCommand
import math

import robot


class HighGoalShootCommand(BaseShootCommand):
    def __init__(self):
        super().__init__()

        self.startDistance = robot.shooter.startDistance  # meters from the target

    def initialize(self):
        self.resetShooterAtRPM()
        self.setFarHoodPosition()

    def execute(self):
        currentDistance = (
            robot.limelight.fDistance
            if robot.drivetrain.isLimelightLockEnabled()
            else robot.limelight.getDistance()
        )

        distance = currentDistance - self.startDistance

        # Calculate new rpm and hood values based on the current distance
        [rpm1, rpm2] = robot.shooter.calculateRPMsFromDistance(distance)

        self.setRPMs(rpm1, rpm2)

        # Run core class methods
        if robot.limelight.isVisionTargetDetected():
            self.shootIfShooterAtSpeed()
