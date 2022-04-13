from commands2 import CommandBase

import robot


class BaseShootCommand(CommandBase):
    def __init__(self):
        super().__init__()

        self.addRequirements([robot.shooter, robot.hood])
        self.shooterRPMTolerance = 40

        # These values should be set within the initialize
        # or execute of the child command
        self.rpm1 = 0
        self.rpm2 = 0

    # This is a demo execute command
    # def execute(self):
    #     self.shootIfShooterAtSpeed()

    def end(self, interrupted):
        robot.shooter.stopShooter()
        robot.ballsystem.stopConveyor()
        robot.ballsystem.stopChamber()
        robot.drivetrain.disableShootMode()
        robot.limelight.stopTakingSnapshots()

    def configureLimelightPipeline(self):
        robot.limelight.setPipeline(1)

    def resetShooterAtRPM(self):
        self.shooterAtRPM = False

    def setRPMs(self, rpm1, rpm2):
        self.rpm1 = rpm1
        self.rpm2 = rpm2

        robot.shooter.setRPM(self.rpm1, self.rpm2)

    def setFarHoodPosition(self):
        robot.hood.setFarHoodPosition()

    def setCloseHoodPosition(self):
        robot.hood.setCloseHoodPosition()

    def shootIfShooterAtSpeed(self):
        targetRPM1 = self.rpm1
        targetRPM2 = self.rpm2

        # Check if both of the shooter wheels are up to speed
        shooterAtSpeed1 = (
            abs(robot.shooter.getRPM1() - targetRPM1) <= self.shooterRPMTolerance
        )
        shooterAtSpeed2 = (
            abs(robot.shooter.getRPM2() - targetRPM2) <= self.shooterRPMTolerance
        )

        if not self.shooterAtRPM:
            self.shooterAtRPM = shooterAtSpeed1 and shooterAtSpeed2

        # Move the ball through the chamber if the shooter is up to speed
        if self.shooterAtRPM:
            robot.ballsystem.forwardChamber()
            robot.ballsystem.forwardConveyor()
        else:
            robot.ballsystem.stopChamber()
            robot.ballsystem.stopConveyor()
