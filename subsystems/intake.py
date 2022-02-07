from .cougarsystem import *

import ports

from ctre import WPI_TalonSRX, NeutralMode, ControlMode


class Intake(CougarSystem):
    """Controls the intake."""

    def __init__(self):
        super().__init__("Intake")

        self.motor = WPI_TalonSRX(ports.intake.motorID)

        self.motor.setNeutralMode(NeutralMode.Brake)
        self.motor.setInverted(True)

        # self.speed = 1
        self.bindVariable("speed", "Intake Speed", 1.0)

    def periodic(self):
        """
        Loops when nothing else is running in
        this subsystem. Do not call this!
        """
        self.feed()

    def intakeBalls(self):
        """
        Run the intake so the balls move
        forwards.
        """
        self.move(self.speed)

    def outtakeBalls(self):
        """
        Reverse the intake so the balls
        move backwards.
        """
        self.move(-self.speed)

    def move(self, speed):
        """
        Basic move method to set custom speed to the motor.
        """
        self.motor.set(ControlMode.PercentOutput, speed)

    def stop(self):
        self.motor.stopMotor()
