from commands2 import CommandBase

import robot


class HighGoalShootCommand(CommandBase):
    def __init__(self):
        super().__init__()

        self.addRequirements(robot.shooter)
        self.shooterRPMTolerance = 150

        self.rpm1 = 0
        self.rpm2 = 0

    def initialize(self):
        robot.shooter.setRPM(self.rpm1, self.rpm2)
        robot.ballsystem.forwardConveyor()

    def execute(self):
        self.shootIfShooterAtSpeed()

    def end(self, interrupted):
        robot.shooter.stopShooter()
        robot.ballsystem.stopConveyor()
        robot.ballsystem.stopChamber()

    def shootIfShooterAtSpeed(self):
        shooterAtSpeed1 = (
            abs(robot.shooter.getRPM1() - self.rpm1) <= self.shooterRPMTolerance
        )
        shooterAtSpeed2 = (
            abs(robot.shooter.getRPM2() - self.rpm2) <= self.shooterRPMTolerance
        )

        if shooterAtSpeed1 and shooterAtSpeed2:
            robot.ballsystem.forwardChamber()
