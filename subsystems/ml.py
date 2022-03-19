from .cougarsystem import CougarSystem
import ports
import math

import constants

from wpilib import DriverStation

from wpimath.geometry import Pose2d

from networktables import NetworkTables


class ML(CougarSystem):
    """Describe what this subsystem does."""

    def __init__(self):
        super().__init__("ML")
        self.nt = NetworkTables.getTable("ML")

        self.bindVariable("turnP", "turn P", 1)
        self.bindVariable("turnI", "turn I", 0)
        self.bindVariable("turnD", "turn D", 0)
        self.bindVariable("maxVel", "velocity", 1)
        self.bindVariable("maxAcc", "acceleration", 2)
        self.bindVariable("stopSize", "ball stopping size", 13000)
        # self.driveTable = NetworkTables.getTable("DriveTrain")

    def periodic(self):
        """
        Loops when nothing else is running in
        this subsystem. Do not call this!
        """
        self.feed()

    def isTargetAcquired(self):
        """
        Returns whether or not a target is actively detected by the ml system.
        """
        return self.get("targetAcquired")

    def setFilterColor(self, color):
        """
        Sets the color to filter for.
        """
        self.put("filterColor", color)

    def getTargetColor(self):
        """
        Returns the color of the currently selected target (DriverStation.Alliance)
        """
        rawTargetColor = self.get("targetColor")

        if rawTargetColor == "red":
            return DriverStation.Alliance.kRed
        elif rawTargetColor == "blue":
            return DriverStation.Alliance.kBlue
        else:
            return DriverStation.Alliance.kInvalid

    def getX(self):
        """
        Returns the x position of the largest/closest ball
        """
        return self.get("targetX")

    def getY(self):
        """
        Returns the y position of the largest/closest ball
        """
        return self.get("targetY")

    def getArea(self):
        """
        Returns the area of the largest/closest ball
        """
        return self.get("targetArea")

    def getResX(self):
        """
        Returns the x resolution of the ml device
        """
        return self.get("resolutionX")

    def getResY(self):
        """
        Returns the y resolution of the ml device
        """
        return self.get("resolutionY")

    def getXNormalized(self):
        """Returns a value between -1 (left) and 1 (right) for where the ball is on the x axis"""
        xPosition = self.getX()
        xResolution = self.getResX()

        # normalizes and then moves
        xNormalized = xPosition / xResolution * 2 - 1

        return xNormalized

    def getYNormalized(self):
        """Returns a value between -1 (top) and 1 (bottom) for where the ball is on the x axis"""
        yPosition = self.getY()
        yResolution = self.getResY()

        # normalizes and then moves
        yNormalized = -(yPosition / yResolution * 2 - 1)

        return yNormalized

    def getXAngle(self):
        """calculates the angle of the ball to the robot x"""
        horizontalAngle = -self.getXNormalized() / constants.ml.horizontalFieldOfView
        return horizontalAngle  # counter clockwise positive

    def getYAngle(self):
        """calculates the angle of the ball to the robot y"""
        verticalAngle = -self.getYNormalized() / constants.ml.verticalFieldOfView
        return verticalAngle  # up positive

    def getBallDistance(self):
        # this is equal to the sqrt of the ball area at one meter according to the camera
        sqrtOfBallSize = 76
        return sqrtOfBallSize / math.sqrt(self.getArea())
