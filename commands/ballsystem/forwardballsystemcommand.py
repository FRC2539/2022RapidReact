from commands2 import CommandBase

import wpilib

import robot


class ForwardBallSystemCommand(CommandBase):

    """
    Run the conveyor and chamber/launcher forward.
    """

    def __init__(self, useLights=True):
        super().__init__()
        self.addRequirements([robot.ballsystem, robot.lights])
        self.timer = wpilib.Timer()
        self.lightsOn = False
        self.useLights = useLights

    def initialize(self):
        self.timer.start()

    def execute(self):
        self.ballPresentStopping()

        if self.useLights:
            self.blinkBallColor()

    def end(self, interrupted):
        robot.ballsystem.stopConveyor()
        robot.ballsystem.stopChamber()

        if self.useLights:
            finalColor = self.allianceToColor(robot.ballsystem.getChamberBallColor())
            robot.lights.set(finalColor)

    def ballPresentStopping(self):
        chamberBall = robot.ballsystem.isChamberBallPresent()
        conveyorBall = robot.ballsystem.isConveyorBallPresent()

        # Manage the chamber using the chamber sensor
        if not chamberBall:
            robot.ballsystem.forwardChamber()
        else:
            robot.ballsystem.stopChamber()

        # Manage the conveyor using the conveyor and chamber sensor
        if conveyorBall and not chamberBall:
            robot.ballsystem.forwardConveyor()
        elif not conveyorBall:
            robot.ballsystem.forwardConveyor()
        else:
            robot.ballsystem.stopConveyor()

    def blinkBallColor(self):
        blinkColor = self.allianceToColor(robot.ballsystem.getChamberBallColor())

        if self.timer.hasElapsed(0.25):
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
