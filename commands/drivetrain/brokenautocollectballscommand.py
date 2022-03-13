from commands2 import CommandBase

import robot

import constants

from wpimath.geometry import Translation2d, Rotation2d, Pose2d
from wpilib import Timer
from wpimath.kinematics import ChassisSpeeds
from wpimath.controller import ProfiledPIDController, ProfiledPIDControllerRadians
from wpimath.trajectory import TrapezoidProfileRadians
from wpimath.filter import MedianFilter
import constants
import math
import wpilib


class AutoCollectBallsCommand(CommandBase):
    """Turns towards ball and then moves at it. The intake is running during the entire command."""

    def __init__(self, endOnBallPickup=False, pickupTwo=False):
        super().__init__()

        self.addRequirements(robot.drivetrain)

        self.endOnBallPickup = endOnBallPickup
        self.pickupTwo = pickupTwo

        # sets the min and max speed the robot will spin at
        self.maxRotationSpeed = constants.drivetrain.angularSpeedLimit / 8
        self.minRotationSpeed = constants.drivetrain.angularSpeedMinimum

        # sets the speed the robot will move forwards
        self.maxLinearSpeed = constants.drivetrain.speedLimit / 4

        self.pickupSpeed = self.maxLinearSpeed / 2

        # the angle the robot will turn to be within
        self.turningRadianTolerance = 0.2
        # the angle the robot will move when it is within
        self.movingRadianTolerance = 0.4

        self.turningConstraints = TrapezoidProfileRadians.Constraints(
            self.maxRotationSpeed, 2
        )
        # creates a timer for later letting the robot run slightly after losing sight of the ball
        self.timer = Timer()

        self.lightsTimer = Timer()

        self.timerStarted = False

        self.lightsOn = False

        self.angleController = ProfiledPIDControllerRadians(
            1, 0, 0, self.turningConstraints
        )

        self.angleController.setTolerance(self.turningRadianTolerance)
        # self.forwardController = ProfiledPIDController(1,0,0)

    def initialize(self):
        self.angleController.setPID(robot.ml.turnP, robot.ml.turnI, robot.ml.turnD)
        self.angleController.setConstraints(
            TrapezoidProfileRadians.Constraints(robot.ml.maxVel, robot.ml.maxAcc)
        )
        self.angleController.reset(self.getRobotAngle())
        self.angleController.setGoal(-self.getBallAngle() + self.getRobotAngle())

        self.timer.reset()
        self.timer.start()
        self.lightsTimer.reset()
        self.lightsTimer.start()

        self.ballInChamber = robot.ballsystem.isChamberBallPresent()
        self.ballInConveyor = robot.ballsystem.isConveyorBallPresent()

    def execute(self):
        self.blinkBallColor()
        robot.drivetrain.setChassisSpeeds(ChassisSpeeds(0, 0, self.calcRotationSpeed()))

    def isFinished(self):
        conveyorBall = robot.ballsystem.isConveyorBallPresent()
        chamberBall = robot.ballsystem.isChamberBallPresent()

        if not self.endOnBallPickup:
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
        # resets the timer everytime a target is aquired for later
        if robot.ml.isTargetAcquired():
            self.timerStarted = True
            self.timer.reset()
            self.timer.start()

        # lets the robot continue to move forward for 0.2 seconds after losing sight of the ball
        if (
            not robot.ml.isTargetAcquired()
            and not self.timer.hasElapsed(0.05)
            and self.timerStarted
        ):
            return self.pickupSpeed

        # otherwise, if 0.2 seconds has passed, the robot is stopped
        if not robot.ml.isTargetAcquired():
            return 0

        velocity = 0

        # keeps the velocity zero if the robot is not pointing close enough to the ball
        if self.angleController.getPositionError() <= self.movingRadianTolerance:
            velocity = self.maxLinearSpeed

        return velocity

    def calcRotationSpeed(self):
        calculatedRotationSpeed = self.angleController.calculate(self.getRobotAngle())
        return calculatedRotationSpeed

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

    def getRobotAngle(self):
        a = robot.drivetrain.navX.getAngle() / 180 * math.pi
        print(f"robot angle = {a}")
        return a

    def getBallAngle(self):
        return robot.ml.getXAngle()
