from .cougarsystem import *

import ports
import constants
import math

from rev import CANSparkMax


class Hood(CougarSystem):
    """Controls the robot's hood."""

    def __init__(self):
        super().__init__("Hood")

        # Define the motor object.
        self.motor = CANSparkMax(ports.hood.motorID, CANSparkMax.MotorType.kBrushless)
        self.motor.setIdleMode(CANSparkMax.IdleMode.kBrake)
        self.motor.burnFlash()

        # Get the motor's PID controller.
        self.controller = self.motor.getPIDController()

        # Get the motor's encoder
        self.encoder = self.motor.getEncoder()
        self.encoder.setPosition(0)
        # self.encoder.setPositionConversionFactor(  # See the constant for an explanation
        #     20.0
        #     # constants.hood.positionConversionFactor
        # )

        # Adjust the hood's PID control values.
        self.controller.setP(0.01)
        self.controller.setI(0)
        self.controller.setD(0)
        self.controller.setFF(0)
        self.controller.setIZone(0)
        # self.controller.setOutputRange(-1, 1)

        # The hood's max and min angle.
        self.maxAngle = constants.hood.maxAngle
        self.minAngle = constants.hood.minAngle

        # The percent to run the hood motor at by default.
        # self.speed = constants.hood.percentOutputSpeed
        self.bindVariable("speed", "Speed Percent", 0.5)

        # Constantly updates the hood's status.
        self.constantlyUpdate("Hood Moving", lambda: self.motor.get() != 0)
        self.constantlyUpdate("Hood Position", self.getPosition)

        self.lowGoalAngle = 16  # 10
        # self.highGoalAngle = 13
        self.bindVariable("highGoalAngle", "High Hood Angle", 13)

        self.bindVariable("testAngle", "Test Angle", 12)

        self.hoodAngleOffset = 0
        self.hoodAngleStep = 1

        self.bindVariable("startHoodAngle", "Start Hood Angle", 25)
        self.bindVariable("hoodMultiplier", "Hood Multiplier", 9)

        self.bindVariable("kP", "kP", 0.065)
        self.bindVariable("maxAdjust", "maxAdjust", 0.9)

    def periodic(self):
        """
        Loops when nothing else is running in
        this subsystem. Do not call this!
        """
        self.feed()

    def increaseHoodOffset(self):
        self.hoodAngleOffset += self.hoodAngleStep

    def decreaseHoodOffset(self):
        self.hoodAngleOffset -= self.hoodAngleStep

    def getPosition(self):
        """
        Returns the position of the hood's encoder
        in degrees, -360 to 360 (not physically possible).
        """
        return (
            self.encoder.getPosition() * 20 * -1
        )  # Motor runs backwards when positive

    def setPosition(self, position):
        """
        Uses the smart motor controller PID capabilities to set
        the hood to a specified position.

        Based on the current setup the position should be an angle from -45 to 45.
        """
        # if not self.angleIsWithinBounds(position):
        #     return

        self.controller.setReference(position, CANSparkMax.ControlType.kPosition, 0, 0)

    def setPercent(self, speed):
        """
        A 'raw' move method that moves the motor unconstrained at a constant speed
        """
        self.motor.set(speed)

    def up(self):
        """
        Raise the hood.
        """
        self.move(self.speed)

    def down(self):
        """
        Lower the hood.
        """
        self.move(-self.speed)

    def move(self, speed):
        """
        Safely moves the hood with a given
        speed.
        """
        if self.isInAngleBounds(speed):
            self.motor.set(-1 * speed)  # Motor runs backwards when positive
        else:
            self.stop()

    def rawMove(self, speed):
        self.motor.set(speed)

    def isInAngleBounds(self, speed=0):
        """
        Can the hood move the way you want it to
        (based off of the sign of the speed)? It returns
        true or false depending on if it can move that specific
        direction.
        """
        if speed > 0:
            return self.isUnderMaxAngle()
        elif speed < 0:
            return self.isAboveMinAngle()
        else:
            return self.isUnderMaxAngle() and self.isAboveMinAngle()

    def angleIsWithinBounds(self, angle):
        """
        Checks if the given angle is a valid angle for the hood
        """
        return self.minAngle <= angle <= self.maxAngle

    def isUnderMaxAngle(self):
        """
        Is the hood within the max angle?
        """
        return self.getPosition() <= self.maxAngle

    def isAboveMinAngle(self):
        """
        Is the hood above the min angle?
        """
        return self.minAngle <= self.getPosition()

    def stop(self):
        """
        Stops the hood motor.
        """
        self.motor.stopMotor()
        # self.sendMessage("Hood Stopped!")

    def calculateHoodAngleFromDistance(self, distance):
        """
        Uses the distance from the target to calculate the hood angle.
        """
        return self.startHoodAngle + self.hoodMultiplier * distance

    def getAdjustSpeed(self, position):
        currentPosition = self.getPosition()
        distance = abs(currentPosition - position)
        direction = math.copysign(1, position - currentPosition)

        adjust = distance * self.kP

        return min(self.maxAdjust, max(adjust, 0.03)) * direction
