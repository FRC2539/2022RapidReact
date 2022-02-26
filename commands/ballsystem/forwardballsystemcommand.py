from commands2 import CommandBase

import wpilib

import robot


class ForwardBallSystemCommand(CommandBase):

    """
    Run the conveyor and chamber/launcher forward.
    """

    def __init__(self):
        super().__init__()
        self.addRequirements([robot.ballsystem, robot.lights])
        self.timer = wpilib.Timer()
        self.lightsOn = False
        self.lightsColor = wpilib.DriverStation.Alliance.kInvalid

    def initialize(self):
        self.timer.start()

    def execute(self):
        robot.ballsystem.forwardConveyor()
        self.ballPresentStopping()
        self.blinkBallColor()

    def end(self, interrupted):
        robot.ballsystem.stopConveyor()
        robot.ballsystem.stopChamber()

    def ballPresentStopping(self):
        if not robot.ballsystem.isChamberBallPresent():
            robot.ballsystem.forwardChamber()
        else:
            robot.ballsystem.stopChamber()

    def blinkBallColor(self):
        ballColor = robot.ballsystem.getChamberBallColor()

        if ballColor == wpilib.DriverStation.Alliance.kRed:
            blinkColor = robot.lights.colors["red"]
        elif ballColor == wpilib.DriverStation.Alliance.kBlue:
            blinkColor = robot.lights.colors["blue"]
        else:
            blinkColor = robot.lights.colors["yellow"]

        if self.timer.hasElapsed(0.25):
            self.lightsOn = not self.lightsOn
            self.timer.reset()

        if self.lightsOn:
            robot.lights.set(blinkColor)
        else:
            robot.lights.off()
