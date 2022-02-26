from commands2 import CommandBase

import robot


class BaseShootCommand(CommandBase):
    def __init__(self):
        super().__init__()

        self.addRequirements([robot.shooter, robot.hood])
        self.shooterRPMTolerance = 75
        self.hoodTolerance = 0.3

        # These values should be set within the initialize
        # or execute of the child command
        self.rpm1 = 0
        self.rpm2 = 0
        self.hoodAngle = 0

    # This is a demo execute command
    # def execute(self):
    #     self.shootIfShooterAtSpeed()
    #     self.updateHoodPosition()

    def end(self, interrupted):
        robot.shooter.stopShooter()
        robot.ballsystem.stopConveyor()
        robot.ballsystem.stopChamber()
        robot.hood.stop()

    def setRPMs(self, rpm1, rpm2):
        self.rpm1 = rpm1
        self.rpm2 = rpm2

        robot.shooter.setRPM(self.rpm1, self.rpm2)

    def setHoodPosition(self, position):
        self.hoodAngle = position

    def updateHoodPosition(self):
        # Calculate the hood angle offset
        distance = abs(self.hoodAngle - robot.hood.getPosition())

        # Calculate a speed based on the hood angle offset
        speed = 0.1 if distance > 2 else 0.03
        direction = 1 if (self.hoodAngle - robot.hood.getPosition()) >= 0 else -1

        # Move the hood as long as it is not yet within the tolerance
        if abs(self.hoodAngle - robot.hood.getPosition()) > self.hoodTolerance:
            robot.hood.move(speed * direction)
        else:
            robot.hood.stop()

    def shootIfShooterAtSpeed(self):
        # Check if both of the shooter wheels are up to speed
        shooterAtSpeed1 = (
            abs(robot.shooter.getRPM1() - self.rpm1) <= self.shooterRPMTolerance
        )
        shooterAtSpeed2 = (
            abs(robot.shooter.getRPM2() - self.rpm2) <= self.shooterRPMTolerance
        )

        # Move the ball through the chamber if the shooter is up to speed
        if shooterAtSpeed1 and shooterAtSpeed2:
            robot.ballsystem.forwardChamber()
            robot.ballsystem.forwardConveyor()
        else:
            robot.ballsystem.stopChamber()
            robot.ballsystem.stopConveyor()
