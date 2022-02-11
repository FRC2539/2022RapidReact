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

    def getX(self):
        """
        Returns the current latency of the limelight (in ms)
        """
        return self.get("x")

    def getY(self):
        """
        Returns the current latency of the limelight (in ms)
        """
        return self.get("y")

    def getArea(self):
        """
        Returns the current latency of the limelight (in ms)
        """
        return self.get("area")

    def getResX(self):
        """
        Returns the current latency of the limelight (in ms)
        """
        return self.get("resolutionx")

    def getResY(self):
        """
        Returns the current latency of the limelight (in ms)
        """
        return self.get("resolutiony")
