from .cougarsystem import *

import ports
import robot
import constants
import math

from wpimath.geometry import Pose2d

from networktables import NetworkTables


class Limelight(CougarSystem):
    """Subsystem for interacting with the limelight."""

    def __init__(self):
        super().__init__("limelight")

        self.nt = NetworkTables.getTable("limelight")

        self.setPipeline(constants.limelight.defaultPipeline)

        # The deadband for whether we are aimed or not.
        self.aimedDeadband = 0.25
        self.put("aimedDeadband", self.aimedDeadband)

        self.currentDistance = 0

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
        # TODO test variable binding
        self.updateXOffset()
        self.updateYOffset()
        self.updateXOffsetStep()
        self.updateYOffsetStep()

        # Limelight aim config
        self.bindVariable("limelightP", "Aim P", 0.04)
        self.bindVariable("percentErrorThreshold", "Percent Threshold", 0.14)
        self.bindVariable("minTurnPercent", "Minimum Turn", 0.14)
        self.bindVariable("aimedThreshold", "Aimed Threshold", 0.3)

        self.bindVariable("kP", "kP", 0.02)
        self.bindVariable("maxAdjust", "maxAdjust", 0.35)

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

        self.updateCurrentDistance()

        self.put("Distance", self.currentDistance)

    def updateCurrentDistance(self):
        self.currentDistance = self.calculateDistance()

    def getDistance(self):
        return self.currentDistance

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

        try:
            yOffset = self.getY()
        except (TypeError):
            yOffset = 0

        totalAngle = constants.limelight.limelightAngle + math.radians(yOffset)

        return constants.limelight.heightOffset / math.tan(totalAngle)

    def estimateLimelightPose(self):
        """
        Calculates the limelight pose relative to the target.
        """
        return Pose2d(
            -self.getDistance(),  # Treats the target as being "forward"
            0,  # Given that the target is a circle, there is no real sideways offset
            math.radians(robot.limelight.getX()),
        )

    def estimateShooterPose(self):
        """
        Calculates the shooter pose relative to the target.

        It uses the limelight pose estimate and transforms it
        to create the estimate.
        """
        return self.estimateLimelightPose().transformBy(
            constants.limelight.limelightToShooterTransform
        )

    def isAimed(self):
        """
        Returns true if the limelight is on target.

        Note: Best used when the limelight moves with a shooter or mechanism.
        """
        return (
            abs(self.getY()) < self.aimedDeadband
            and abs(self.getX()) < self.aimedDeadband
        )

    def calculateFutureRobotVector(self):
        [robotX, robotY] = robot.drivetrain.calculateRobotRelativeVector()

        normalizedX, normalizedY = robotX * 0.02, robotY * 0.02

        theta = math.radians(self.getX())

        limelightX, limelightY = self.currentDistance * math.cos(
            theta
        ) + self.currentDistance * math.sin(theta)

        return [limelightX + normalizedX, limelightY + normalizedY]

    def calculateTurnPercent(self):
        xOffset = self.getX()  # Returns an angle

        if abs(xOffset) < self.aimedThreshold:
            return 0

        try:
            xPercentError = (
                xOffset * self.limelightP
            )  # This value is found experimentally
        except (TypeError):
            xPercentError = 0
            print("\nERROR: Limelight is broken/unplugged \n")

        if abs(xPercentError) > self.percentErrorThreshold:
            xPercentError = math.copysign(self.minTurnPercent, xPercentError)

        return xPercentError
