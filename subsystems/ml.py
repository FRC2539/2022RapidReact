from .cougarsystem import CougarSystem
import ports
import math

import constants

from wpimath.geometry import Pose2d

from networktables import NetworkTables


class ML(CougarSystem):
    """Describe what this subsystem does."""

    def __init__(self):
        super().__init__("ML")
        self.nt = NetworkTables.getTable("ML")
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
