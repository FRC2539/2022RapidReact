from .cougarsystem import *

import wpilib
import math
import ports
import constants

from rev import CANSparkMax, MotorType, ControlType, IdleMode


class Hood(CougarSystem):
    """Controls the robot's hood."""

    def __init__(self):
        super().__init__("Hood")

        # Define the motor object.
        self.motor = CANSparkMax(ports.hood.motorID, MotorType.kBrushless)
        self.motor.setIdleMode(IdleMode.kBrake)
        self.motor.burnFlash()

        # Get the motor's PID controller.
        self.controller = self.motor.getPIDController()

        # Get the motor's encoder
        self.encoder = self.motor.getEncoder()
        self.encoder.setPositionConversionFactor(  # See the constant for an explanation
            constants.hood.positionConversionFactor
        )

        # Adjust the hood's PID control values.
        self.controller.setP(constants.hood.kP, 0)
        self.controller.setI(0, 0)
        self.controller.setD(0, 0)
        self.controller.setFF(0, 0)
        self.controller.setIZone(0, 0)

        # The hood's max and min angle.
        self.maxAngle = constants.hood.maxAngle
        self.minAngle = constants.hood.minAngle

        # The percent to run the hood motor at by default.
        self.speed = 0.3  # 30% percent.

        # Constantly updates the hood's status.
        self.constantlyUpdate("Hood Moving", lambda: self.motor.get() != 0)
        self.constantlyUpdate("Hood Position", self.getPosition)

    def periodic(self):
        """
        Loops when nothing else is running in
        this subsystem. Do not call this!
        """
        self.feed()

    def getPosition(self):
        """
        Returns the position of the hood's encoder
        in degrees, -360 to 360 (not physically possible).
        """
        return self.encoder.getPosition() * 360

    def setPosition(self, position):
        """
        Uses the smart motor controller PID capabilities to set
        the hood to a specified position.

        Based on the current setup the position should be an angle from -45 to 45.
        """
        if not self.angleIsWithinBounds(position):
            return

        self.controller.setReference(position, ControlType.kPosition)

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
            self.motor.set(speed)
        else:
            self.stop()

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
        self.sendMessage("Hood Stopped!")