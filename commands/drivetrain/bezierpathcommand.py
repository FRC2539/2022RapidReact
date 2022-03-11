from commands2 import CommandBase

import math, os, signal

import robot

# Counts how many iterations we've done in align().
loopCount = 0


def align(angle):
    global loopCount

    loopCount += 1

    count = 0
    if angle < 0:
        angle += 360
    for a in robot.drivetrain.getModuleAngles():
        if abs(a - angle) < 3 or (
            a > 357 and abs(angle) < 2
        ):  # The 'or' is a temporary fix.
            count += 1

    if count >= 3:  # TODO: Tune the max loop count.
        return True

    return False


class BezierPathCommand(CommandBase):
    def __init__(self, points: list, speed=1, stopWhenDone=True):
        """
        This command will make the robot follow any Bezier
        curve. Give the points in inches please!

        points - first and last points are anchor points
                 second and third points are corresponding control points
        """

        super().__init__()

        self.addRequirements(robot.drivetrain)

        self.getPosition = robot.drivetrain.getHigherBezierPosition

        # Use a quadratic Bezier.
        if len(points) == 3:
            self.getSlope = robot.drivetrain.getQuadraticBezierSlope
            self.getLength = robot.drivetrain.getQuadraticBezierLength

        # Use a cubic Bezier.
        elif len(points) == 4:
            self.getSlope = robot.drivetrain.getCubicBezierSlope
            self.getLength = robot.drivetrain.getHigherBezierLength

        # Use a higher Bezier.
        else:
            self.getSlope = robot.drivetrain.getHigherBezierSlope
            self.getLength = robot.drivetrain.getHigherBezierLength

        # Define our variables.
        self.points = points
        self.speed = speed
        self.stopWhenDone = stopWhenDone
        self.kP = 0.0285

        # Set the 't' of the parametric function.
        self.t = 0.2

        # Calculate/estimate the curve length.
        self.curveLength = self.getLength(self.points)

    def initialize(self):
        # Reset out variables.
        self.t = 0
        loopCount = 0

        # Get the start position of the back left module.
        self.startPos = robot.drivetrain.getPositions()[2]

        slopeComponents = self.getSlope(self.points, self.t)
        angle = math.atan2(slopeComponents[0], slopeComponents[1]) * 180 / math.pi + 90

        if angle > 180:
            angle -= 360

        # Set and wait for the module angles to go to the right position.
        robot.drivetrain.setUniformModuleAngle(angle)

        while not align(angle):
            pass

        # Set the drive speed.
        robot.drivetrain.setUniformModuleSpeed(self.speed)

    def execute(self):
        # 'self.t' is the percent of the curve which we have traversed. We use it to calculate our heading.
        self.t = abs(
            (robot.drivetrain.getPositions()[2] - self.startPos) / self.curveLength
        )

        # Calculate and set the angle.
        slopeComponents = self.getSlope(self.points, self.t)
        angle = math.atan2(slopeComponents[0], slopeComponents[1]) * 180 / math.pi + 90

        if angle > 180:
            angle -= 360

        if self.t != 1:
            robot.drivetrain.setUniformModuleAngle(angle)

        # Adjust for angle.
        if abs(angle) < 90:
            speedOffset = robot.drivetrain.getAngleTo(0) * -self.kP
        else:
            speedOffset = robot.drivetrain.getAngleTo(0) * self.kP

        robot.drivetrain.setSpeeds(
            [
                self.speed + speedOffset,
                self.speed - speedOffset,
                self.speed + speedOffset,
                self.speed - speedOffset,
            ]
        )

    def isFinished(self):
        # We are done when we have travelled 100% of the curve.
        return self.t >= 1

    def end(self, interrupted):
        # Stop moving when we're done.
        if self.stopWhenDone:
            robot.drivetrain.stop()
