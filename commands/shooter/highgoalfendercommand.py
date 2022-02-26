from commands2 import CommandBase

import robot


class HighGoalFenderCommand(CommandBase):
    def __init__(self):
        super().__init__()

        self.addRequirements([robot.shooter, robot.hood])
        self.shooterRPMTolerance = 100
        self.hoodTolerance = 0.5

    def initialize(self):
        self.rpm1 = robot.shooter.highGoalRPM1
        self.rpm2 = robot.shooter.highGoalRPM2
        self.hoodAngle = 11

        robot.shooter.setRPM(self.rpm1, self.rpm2)
        robot.ballsystem.forwardConveyor()

    def execute(self):
        self.shootIfShooterAtSpeed()

        distance = abs(self.hoodAngle - robot.hood.getPosition())
        speed = 0.1 if distance > 2 else 0.03

        direction = 1 if (self.hoodAngle - robot.hood.getPosition()) >= 0 else -1

        if abs(self.hoodAngle - robot.hood.getPosition()) > self.hoodTolerance:
            robot.hood.move(speed * direction)
        else:
            robot.hood.stop()

    def end(self, interrupted):
        robot.shooter.stopShooter()
        robot.ballsystem.stopConveyor()
        robot.ballsystem.stopChamber()
        robot.hood.stop()

    def shootIfShooterAtSpeed(self):
        shooterAtSpeed1 = (
            abs(robot.shooter.getRPM1() - self.rpm1) <= self.shooterRPMTolerance
        )
        shooterAtSpeed2 = (
            abs(robot.shooter.getRPM2() - self.rpm2) <= self.shooterRPMTolerance
        )

        if shooterAtSpeed1 and shooterAtSpeed2:
            robot.ballsystem.forwardChamber()
        else:
            robot.ballsystem.stopChamber()
