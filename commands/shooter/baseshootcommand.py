from commands2 import CommandBase

import robot


class BaseShootCommand(CommandBase):
    def __init__(self):
        super().__init__()

        self.addRequirements([robot.shooter, robot.hood])
        self.shooterRPMTolerance = 70
        self.hoodTolerance = 0.6

        # These values should be set within the initialize
        # or execute of the child command
        self.rpm1 = 0
        self.rpm2 = 0
        self.hoodAngle = 0

        # Reject balls of the wrong color
        self.rejectMode = False

    # This is a demo execute command
    # def execute(self):
    #     self.shootIfShooterAtSpeed()
    #     self.updateHoodPosition()

    def end(self, interrupted):
        robot.shooter.stopShooter()
        robot.ballsystem.stopConveyor()
        robot.ballsystem.stopChamber()
        robot.hood.stop()

    def configureLimelightPipeline(self):
        robot.limelight.setPipeline(1)

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
        speed = 0.05 if distance > 2 else 0.02
        direction = 1 if (self.hoodAngle - robot.hood.getPosition()) >= 0 else -1

        # Move the hood as long as it is not yet within the tolerance
        if abs(self.hoodAngle - robot.hood.getPosition()) > self.hoodTolerance:
            robot.hood.move(speed * direction)
        else:
            robot.hood.stop()

    def shootIfShooterAtSpeed(self):
        targetRPM1 = self.rpm1 if not self.rejectMode else robot.shooter.rejectRPM1
        targetRPM2 = self.rpm2 if not self.rejectMode else robot.shooter.rejectRPM2

        # Check if both of the shooter wheels are up to speed
        shooterAtSpeed1 = (
            abs(robot.shooter.getRPM1() - targetRPM1) <= self.shooterRPMTolerance
        )
        shooterAtSpeed2 = (
            abs(robot.shooter.getRPM2() - targetRPM2) <= self.shooterRPMTolerance
        )

        hoodAtPosition = abs(self.hoodAngle - robot.hood.getPosition()) <= 1.5

        # Set to reject mode if the ball is the wrong color
        if (
            robot.ballsystem.isChamberBallPresent()
            and robot.ballsystem.getChamberBallColor()
            != robot.ballsystem.getAllianceColor()
        ):
            self.rejectMode = True
        else:
            self.rejectMode = False

        # Switch to reject rpms if we must reject the ball
        if self.rejectMode:
            robot.shooter.setRPM(robot.shooter.rejectRPM1, robot.shooter.rejectRPM2)
        else:
            robot.shooter.setRPM(self.rpm1, self.rpm2)

        # Move the ball through the chamber if the shooter is up to speed
        if shooterAtSpeed1 and shooterAtSpeed2:
            robot.ballsystem.forwardChamber()
            robot.ballsystem.forwardConveyor()
        else:
            robot.ballsystem.stopChamber()
            robot.ballsystem.stopConveyor()
