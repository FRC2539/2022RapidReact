from commands2 import CommandBase

import robot


class RejectCommand(CommandBase):
    def __init__(self):
        super().__init__()

        self.addRequirements(robot.intake)

    def initialize(self):
        robot.ballsystem.backwardAll()
        robot.intake.outtakeBalls()
        robot.shooter.setPercent(-0.3, -0.3)

    def end(self, interrupted):
        robot.intake.stop()
        robot.pneumatics.retractIntake()
        robot.ballsystem.stopChamber()
        robot.ballsystem.stopConveyor()
        robot.shooter.stopShooter()
