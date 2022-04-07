from commands2 import CommandBase

import robot

import constants

from wpilib import Timer
from wpimath.kinematics import ChassisSpeeds
import constants
import wpilib


class AutoCollectBallsCommand(CommandBase):
    """Turns towards ball and then moves at it. The intake is running during the entire command."""

    def __init__(self, endOnBallPickup=True, pickupTwo=True, autonomous=False):
        super().__init__()
        self.addRequirements(robot.drivetrain)

        self.endOnBallPickup = endOnBallPickup
        self.pickupTwo = pickupTwo
        self.autonomous = autonomous

        # sets the speed the robot will move forwards
        # self.maxLinearSpeed = constants.drivetrain.speedLimit / 4 * (2)
        self.maxLinearSpeed = constants.drivetrain.speedLimit / 3

        self.pickupSpeed = self.maxLinearSpeed / 2

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

        self.ballInChamber = robot.ballsystem.isChamberBallPresent()
        self.ballInConveyor = robot.ballsystem.isConveyorBallPresent()

        robot.ml.setFilterColor(self.allianceToRawColor(robot.ml.getTargetColor()))

    def execute(self):
        if (
            robot.ballsystem.isConveyorBallPresent()
            and robot.ballsystem.isChamberBallPresent()
        ):
            return

        self.blinkBallColor()

        robot.drivetrain.setChassisSpeeds(
            ChassisSpeeds(-self.calcForwardVelocity(), -self.calcStrafeVelocity(), 0)
        )

        # print(
        #     f"{robot.ml.getX()=}, {self.getXNormalized()=}, {self.calcRotationSpeed()}"
        # )

    def isFinished(self):
        conveyorBall = robot.ballsystem.isConveyorBallPresent()
        chamberBall = robot.ballsystem.isChamberBallPresent()

        if not self.endOnBallPickup or not self.autonomous:
            return False

        elif not self.pickupTwo:
            return (
                (conveyorBall and chamberBall)
                or (self.ballInChamber and conveyorBall)
                or (not self.ballInChamber and chamberBall)
            )

        return conveyorBall and chamberBall

    def end(self, interrupted):
        robot.drivetrain.stop()

        robot.lights.showTeamColor()

    def calcForwardVelocity(self):
        """calculates the velocity the robot should be moving forwards at. m/s"""

        if self.atBallPickup():
            return 0

        # resets the timer everytime a target is aquired for later
        if robot.ml.isTargetAcquired():
            self.timerStarted = True
            self.timer.reset()
            self.timer.start()

        # Let the robot continue to move forward after losing sight of the ball
        if (
            not robot.ml.isTargetAcquired()
            and not self.timer.hasElapsed(0.05)
            and self.timerStarted
        ):
            return self.pickupSpeed

        # Stop if no ball is currently detected
        if not robot.ml.isTargetAcquired():
            return 0

        xOffset = abs(self.getXAngle())

        return self.maxLinearSpeed * (1 - xOffset)  # * (self.getYNormalized() + 1)

    def calcStrafeVelocity(self):
        if self.atBallPickup():
            return 0

        # Stop if no ball is currently detected
        if not robot.ml.isTargetAcquired():
            return 0

        velocity = 0

        xOffset = self.getXAngle()

        # keeps the velocity zero if the robot is not pointing close enough to the ball
        if abs(xOffset) >= self.movingRadianTolerance:
            velocity = (self.maxLinearSpeed * 0.55) * xOffset

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

    def allianceToRawColor(self, alliance):

        if alliance == wpilib.DriverStation.Alliance.kRed:
            blinkColor = "red"
        elif alliance == wpilib.DriverStation.Alliance.kBlue:
            blinkColor = "blue"
        else:
            blinkColor = "yellow"

        return blinkColor

    def atBallPickup(self):
        """checks if the robot is at the proper ball pickup location"""

        if robot.ml.getArea() > robot.ml.stopSize:
            _atBallPickup = True
        # elif robot.ml.isTargetAcquired():
        #     _atBallPickup = False
        else:
            _atBallPickup = False

        return _atBallPickup
