from .cougarsystem import *

import ports
import robot
import constants
import math

from networktables import NetworkTables


class Limelight(CougarSystem):
    """Subsystem for interacting with the limelight."""

    def __init__(self):
        super().__init__("limelight")

        self.nt = NetworkTables.getTable("limelight")

        self.driveTable = NetworkTables.getTable("DriveTrain")

        self.setPipeline(1)

        # The deadband for whether we are aimed or not.
        self.aimedDeadband = 0.25
        self.put("aimedDeadband", self.aimedDeadband)

        # Grabs the offset.
        constants.limelight.xOffset = self.get("xOffset", constants.limelight.xOffset)
        constants.limelight.yOffset = self.get("yOffset", constants.limelight.yOffset)

        # Pulls the original, or default, step from the nt or constants file.
        constants.limelight.xOffsetStep = self.get(
            "xOffsetStep", constants.limelight.xOffsetStep
        )
        constants.limelight.yOffsetStep = self.get(
            "yOffsetStep", constants.limelight.yOffsetStep
        )

        # Creates the initial nt values.
        self.updateXOffset()
        self.updateYOffset()
        self.updateXOffsetStep()
        self.updateYOffsetStep()

    def periodic(self):
        """
        Loops when nothing else is running in
        this subsystem. Do not call this!
        """
        self.feed()

        # Constantly updates the offsetStep.
        constants.limelight.xOffsetStep = self.get(
            "xOffsetStep", constants.limelight.xOffsetStep
        )
        constants.limelight.yOffsetStep = self.get(
            "yOffsetStep", constants.limelight.yOffsetStep
        )

        self.aimedDeadband = self.get("aimedDeadband", self.aimedDeadband)

    def setPipeline(self, pipeline: int):
        """
        Changes the pipeline of the limelight.
        """
        self.put("pipeline", pipeline)

    def setValue(self, name, val):
        """
        Changes any limelight value, given the key and desired val.
        """
        self.put(name, val)

    def getLatency(self):
        """
        Returns the current latency of the limelight (in ms)
        """
        return self.get("tl")

    def getLatencySeconds(self):
        """
        Returns the current latency of the limelight in seconds
        """
        return self.getLatency() / 1000

    def getRawY(self):
        """
        Returns the raw y-value
        """
        return self.get("ty")

    def getY(self):
        """
        Return the y-value 
        """
        return self.get("ty") + constants.limelight.yOffset

    def getRawX(self):
        """
        Returns the raw x-value
        """
        return self.get("tx")

    def getX(self):
        """
        Return the x-value
        """
        return self.get("tx") + constants.limelight.xOffset

    def getA(self):
        """
        Returns the area of the selected target.
        """
        return self.get("ta")

    def getWidth(self):
        """
        Returns the approximate width of the bounding box
        """
        return self.get("thor")

    def getHeight(self):
        """
        Returns the approximate height of the bounding box
        """
        return self.get("tvert")

    def isVisionTargetDetected(self):
        """
        Returns whether or not tape is being detected by the limelight.
        """
        return self.get("tv") == 1

    def takeSnapshots(self):
        """
        Have the limelight take snapshots.
        These can be viewed by connecting to the limelight
        with a USB cable.
        """
        self.put("snapshot", 1)

    def stopTakingSnapshots(self):
        """
        Stops the limelight from continuing to take snapshots.
        """
        self.put("snapshot", 0)

    def updateYOffset(self):
        """
        Publish the y-offset to the dashboard.
        """
        self.put("yOffset", constants.limelight.yOffset)

    def updateXOffset(self):
        """
        Publish the x-offset to the dashboard.
        """
        self.put("xOffset", constants.limelight.xOffset)

    def updateYOffsetStep(self):
        """
        Publish the y-offset step to the dashboard.
        """
        self.put("yOffsetStep", constants.limelight.yOffsetStep)

    def updateXOffsetStep(self):
        """
        Publish the x-offset step to the dashboard.
        """
        self.put("xOffsetStep", constants.limelight.xOffsetStep)

    def calculateDistance(self):
        """
        Calculates the approximate distance to the target.
        
        Notes:
        This algorithm only works with a statically mounted limelight.
        The limelight constants must be correct (limelight height, angle, target height).
        """

        totalAngle = constants.limelight.limelightAngle + math.radians(self.getY())

        return constants.limelight.heightOffset / math.tan(totalAngle)

    def isAimed(self):
        """
        Returns true if the limelight is on target.

        Note: Best used when the limelight moves with a shooter or mechanism.
        """
        return (
            abs(self.getY()) < self.aimedDeadband
            and abs(self.getX()) < self.aimedDeadband
        )
