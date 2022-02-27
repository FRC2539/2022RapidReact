from commands2 import CommandBase

import robot

import constants

from wpimath.geometry import Translation2d, Rotation2d, Pose2d
from wpilib import Timer
from wpimath.kinematics import ChassisSpeeds
import constants
import math
import wpilib


class AutoCollectBallsCommand(CommandBase):
    """Turns towards ball and then moves at it. The intake is running during the entire command."""

    def __init__(self):
        super().__init__()
        self.addRequirements(robot.drivetrain)

        # sets the proportional value in my made up PID controller
        self.reactionSpeed = 0.03

        # sets the min and max speed the robot will spin at
        self.maxRotationSpeed = constants.drivetrain.angularSpeedLimit / 4
        self.minRotationSpeed = constants.drivetrain.angularSpeedMinimum

        # sets the speed the robot will move forwards
        self.maxLinearSpeed = constants.drivetrain.speedLimit / 4

        self.pickupSpeed = self.maxLinearSpeed / 2

        # the angle the robot will turn to be within
        self.turningRadianTolerance = 0.2
        # the angle the robot will move when it is within
        self.movingRadianTolerance = 0.4

        # creates a timer for later letting the robot run slightly after losing sight of the ball
        self.timer = Timer()

        self.lightsTimer = Timer()

        self.timerStarted = False

        self.lightsOn = False

    def initialize(self):
        self.timer.reset()
        self.timer.start()
        self.lightsTimer.reset()
        self.lightsTimer.start()

    def execute(self):
        self.blinkBallColor()

        robot.drivetrain.setChassisSpeeds(
            ChassisSpeeds(-self.calcForwardVelocity(), 0, self.calcRotationSpeed())
        )

        # print(
        #     f"{robot.ml.getX()=}, {self.getXNormalized()=}, {self.calcRotationSpeed()}"
        # )

    def end(self, interrupted):
        robot.drivetrain.stop()

        robot.lights.showTeamColor()

    def calcForwardVelocity(self):
        """calculates the velocity the robot should be moving forwards at. m/s"""
        # resets the timer everytime a target is aquired for later
        if robot.ml.isTargetAcquired():
            self.timerStarted = True
            self.timer.reset()
            self.timer.start()

        # lets the robot continue to move forward for 0.2 seconds after losing sight of the ball
        if (
            not robot.ml.isTargetAcquired()
            and not self.timer.hasElapsed(0.2)
            and self.timerStarted
        ):
            return self.pickupSpeed

        # otherwise, if 0.2 seconds has passed, the robot is stopped
        if not robot.ml.isTargetAcquired():
            return 0

        velocity = 0

        # keeps the velocity zero if the robot is not pointing close enough to the ball
        if abs(self.getXAngle()) <= self.movingRadianTolerance:
            velocity = self.maxLinearSpeed

        return velocity

    def calcRotationSpeed(self):
        """calculates the rotation speed that the robot should be turning in radians. Counterclockwise positive. (I think)"""
        # does not spin towards a ball that doesn't exist
        if not robot.ml.isTargetAcquired():
            return 0

        # sets the rotational velocity of the robot to be proportional to its offset
        velocity = self.getXNormalized() * self.reactionSpeed

        # replaces a bunch of copysigns
        absVel = abs(velocity)

        # limits the speed
        absVel = max(absVel, self.minRotationSpeed)
        absVel = min(absVel, self.maxRotationSpeed)

        # checks if we are in the tolerance and stops
        if abs(self.getXAngle()) <= self.turningRadianTolerance:
            absVel = 0

        velocity = math.copysign(absVel, velocity)

        return velocity

    def getXNormalized(self):
        """Returns a value between -1 (left) and 1 (right) for where the ball is on the x axis"""
        xPosition = robot.ml.getX()
        xResolution = robot.ml.getResX()

        # normalizes and then moves
        xNormalized = xPosition / xResolution * 2 - 1

        return xNormalized

    def getYNormalized(self):
        """Returns a value between -1 (left) and 1 (right) for where the ball is on the x axis"""
        yPosition = robot.ml.getY()
        yResolution = robot.ml.getResY()

        # normalizes and then moves
        yNormalized = -(yPosition / yResolution * 2 - 1)

        return yNormalized

    def getXAngle(self):
        """calculates the angle of the ball to the robot x"""
        horizontalAngle = -self.getXNormalized() / constants.ml.horizontalFieldOfView
        return horizontalAngle  # counter clockwise positive

    def blinkBallColor(self):
        blinkColor = self.allianceToColor(robot.ml.getTargetColor())

        if self.lightsTimer.hasElapsed(0.25):
            self.lightsOn = not self.lightsOn
            self.lightsTimer.reset()

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
