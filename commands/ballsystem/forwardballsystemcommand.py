from commands2 import CommandBase

import wpilib

import robot


class ForwardBallSystemCommand(CommandBase):

    """
    Run the conveyor and chamber/launcher forward.
    """

    def __init__(self, useLights=True, rejectBalls=False):
        super().__init__()
        self.addRequirements([robot.ballsystem, robot.lights])
        self.timer = wpilib.Timer()
        self.lightsOn = False
        self.useLights = useLights
        self.rejectBalls = rejectBalls
        self.manualMode = False

    def initialize(self):
        self.timer.start()

        self.manualMode = robot.ballsystem.manualBallIntake

    def execute(self):
        if not self.manualMode:
            self.ballPresentStopping()
        else:
            self.manualRun()

        if self.useLights:
            self.blinkBallColor()

    def end(self, interrupted):
        robot.ballsystem.stopConveyor()
        robot.ballsystem.stopChamber()
        robot.shooter.stopShooter()

        if self.useLights:
            robot.lights.showTeamColor()

    def manualRun(self):
        robot.ballsystem.forwardChamber()
        robot.ballsystem.forwardConveyor()

    def ballPresentStopping(self):
        chamberBall = robot.ballsystem.isChamberBallPresent()
        conveyorBall = robot.ballsystem.isConveyorBallPresent()

        # Manage the chamber using the chamber sensor
        if not chamberBall:
            robot.ballsystem.forwardChamberIntake()
            robot.shooter.stopShooter()
        # elif (
        #     self.rejectBalls
        #     and robot.ballsystem.getChamberBallColor()
        #     != robot.ballsystem.getAllianceColor()
        #     and chamberBall
        # ):
        #     robot.shooter.setRPM(robot.shooter.rejectRPM1, robot.shooter.rejectRPM2)
        #     robot.ballsystem.forwardChamberIntake()
        else:
            robot.ballsystem.stopChamber()
            robot.shooter.stopShooter()

        # Manage the conveyor using the conveyor and chamber sensor
        if conveyorBall and not chamberBall:
            robot.ballsystem.forwardConveyorIntake()
        elif not conveyorBall:
            robot.ballsystem.forwardConveyorIntake()
        else:
            robot.ballsystem.stopConveyor()

    def blinkBallColor(self):
        blinkColor = self.allianceToColor(robot.ballsystem.getChamberBallColor())

        conveyorBall = robot.ballsystem.isConveyorBallPresent()

        if self.timer.hasElapsed(0.25 if not conveyorBall else 0.1):
            self.lightsOn = not self.lightsOn
            self.timer.reset()

        if self.lightsOn:
            robot.lights.set(blinkColor)
        else:
            robot.lights.off()

    def allianceToColor(self, alliance):

        if alliance == wpilib.DriverStation.Alliance.kRed:
            blinkColor = robot.lights.colors["red"]
        elif alliance == wpilib.DriverStation.Alliance.kBlue:
            blinkColor = robot.lights.colors["blue"]
        else:
            blinkColor = robot.lights.colors["yellow"]

        return blinkColor
