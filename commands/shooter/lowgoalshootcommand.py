from commands2 import CommandBase

from wpilib import Timer

import robot


class LowGoalShootCommand(CommandBase):
    def __init__(self, rpm1, rpm2):
        super().__init__()

        self.rpm1 = rpm1
        self.rpm2 = rpm2
        self.addRequirements(robot.shooter)
        # self.shootTime = 0.5
        self.shooterRPMTolerance = 150

    def initialize(self):
        robot.shooter.setRPM(self.rpm1, self.rpm2)
        robot.ballsystem.forwardConveyor()

        # self.chamberTimer = Timer()
        # self.timerStarted = False
        # self.timerElapsed = False
        # self.chamberBallDetected = False

    def execute(self):
        self.shootIfShooterAtSpeed()
        # self.monitorChamber()

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

    # def monitorConveyor(self):
    #     """Monitors the state of the conveyor and moves balls to the chamber."""
    #     if (true):  # Check if the conveyor sensor is on and the interior sensor is off. If so, run.
    #         robot.ballsystem.forwardConveryor()
    #     else:
    #         robot.ballsystem.stopConveryor()

    # def monitorChamber(self):
    #     """Monitors the state of the chamber sensor and moves balls to the shooter."""
    #     chamberBallDetected = robot.ballsystem.isChamberBallPresent()
    #     timerElapsed = self.chamberTimer.hasElapsed(self.shootTime)

    #     # Check if the shoot time has elapsed
    #     if not timerElapsed:
    #         # Check if internal sensor is on
    #         if chamberBallDetected:
    #             self.chamberTimer.reset()
    #             self.chamberTimer.start()
    #             robot.ballsystem.forwardChamber()
    #         else:
    #             if self.chamberTimer.get() == 0:
    #                 self.chamberTimer.start()
    #             robot.ballsystem.forwardChamber()
    #     # If the interior sensor is off and the timer is finished, stop all.
    #     elif not chamberBallDetected and timerElapsed:
    #         robot.ballsystem.stopChamber()
    #         self.chamberTimer.reset()
