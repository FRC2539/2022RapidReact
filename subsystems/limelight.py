from .cougarsystem import *

import ports
import robot
import constants
import math

from wpimath.geometry import Pose2d

from networktables import NetworkTables

from wpimath.filter import MedianFilter


class Limelight(CougarSystem):
    """Subsystem for interacting with the limelight."""

    def __init__(self):
        super().__init__("limelight")

        self.nt = NetworkTables.getTable("limelight")

        self.setPipeline(constants.limelight.defaultPipeline)

        # The deadband for whether we are aimed or not.
        self.aimedDeadband = 0.05
        self.put("aimedDeadband", self.aimedDeadband)

        self.currentDistance = 0

        self.distanceFilter = MedianFilter(3)

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
        self.bindVariable("limelightP", "Aim P", 0.018)
        # self.bindVariable("percentErrorThreshold", "Percent Threshold", 0.14)
        self.bindVariable("maxTurnPercent", "Maximum Turn", 0.14)
        self.bindVariable("minTurnPercent", "Minimum Turn", 0.132)
        self.bindVariable("aimedThreshold", "Aimed Threshold", 2)

        # self.bindVariable("kP", "kP", 0.02)
        # self.bindVariable("maxAdjust", "maxAdjust", 0.35)

        self.bindVariable("maxOffsetAngle", "maxOffsetAngle", 500)
        self.bindVariable("distanceMult", "distanceMult", 0.25)
        self.bindVariable("tangentMult", "tangentMult", 1)

        self.fDistance = 0

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
        self.currentDistance = self.distanceFilter.calculate(self.calculateDistance())

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
        try:
            return self.get("tx") + constants.limelight.xOffset
        except:
            return 0

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
        return abs(self.getX()) < self.aimedThreshold

    def calculateFutureForAim(self):
        [vx, vy] = robot.drivetrain.getCurrentRelativeVector()

        pvx, pvy = self.secondToPeriod(vx), self.secondToPeriod(vy)

        hubTheta = math.radians(self.getX())

        hubX, hubY = self.currentDistance * math.cos(
            hubTheta
        ), self.currentDistance * math.sin(hubTheta)

        tangentTheta = math.atan2(hubY, hubX) + math.pi / 2

        ux, uy = math.cos(tangentTheta), math.sin(tangentTheta)

        tangentVelocity = self.periodToSecond(ux * pvx + uy * pvy)

        fHubX, fHubY = hubX - pvx, hubY - pvy

        self.fDistance = math.sqrt(math.pow(fHubX, 2) + math.pow(fHubY, 2))

        fTheta = math.atan2(fHubY, fHubX)

        return [tangentVelocity, fTheta]

    def calculateTurnVelocity(self):
        [tangentVelocity, fTheta] = self.calculateFutureForAim()

        # targetTheta = math.radians(self.maxOffsetAngle) * (self.fDistance * self.distanceMult) * (tangentVelocity * self.tangentMult) * -1

        # return self.periodToSecond(fTheta - targetTheta) * -1

        targetTheta = (
            math.radians(self.maxOffsetAngle)
            * (self.fDistance * self.distanceMult)
            * (tangentVelocity * self.tangentMult)
        )

        # turnVelocity = self.periodToSecond(
        #     math.copysign(abs(fTheta - targetTheta), targetTheta)
        # )

        # turnVelocity = math.copysign(abs(fTheta - targetTheta), targetTheta)

        # self.sendMessage(f"Target Theta: {targetTheta}")

        return targetTheta

    def secondToPeriod(self, velocity):
        return velocity * 0.02

    def periodToSecond(self, velocity):
        return velocity / 0.02

    def calculateTurnPercent(self):
        # targetTheta = math.degrees(self.calculateTurnVelocity())

        xOffset = self.getX()  # Returns an angle

        # self.sendMessage(f"{targetTheta} {xOffset}")

        # distance = xOffset if targetTheta == 0 else targetTheta - xOffset

        if abs(xOffset) < self.aimedThreshold:
            return 0

        try:
            xPercentError = (
                xOffset * self.limelightP
            )  # This value is found experimentally
            xPercentError = math.copysign(
                max(abs(xPercentError), self.minTurnPercent), xOffset
            )
        except (TypeError):
            xPercentError = 0
            print("\nERROR: Limelight is broken/unplugged \n")

        if abs(xPercentError) > self.maxTurnPercent:
            xPercentError = math.copysign(self.maxTurnPercent, xPercentError)

        return xPercentError
