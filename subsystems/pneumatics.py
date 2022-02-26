from .cougarsystem import CougarSystem
import ports
import math

from wpilib import Compressor, PneumaticsModuleType, DoubleSolenoid


class Pneumatics(CougarSystem):
    """Controls the robot compressor."""

    def __init__(self):
        super().__init__("Pneumatics")

        pneumaticsModuleType = PneumaticsModuleType.CTREPCM

        # Create the controller for the compressor
        self.compressor = Compressor(
            ports.climberPneumatics.pcmID, pneumaticsModuleType
        )

        self.compressor.enableDigital()

        self.intakeSolenoid = DoubleSolenoid(
            ports.climberPneumatics.pcmID,
            pneumaticsModuleType,
            ports.intake.forwardChannel,
            ports.intake.reverseChannel,
        )

    def periodic(self):
        """
        Loops when nothing else is running in
        this subsystem. Do not call this!
        """
        self.feed()

    def extendIntake(self):
        self.intakeSolenoid.set(DoubleSolenoid.Value.kForward)

    def retractIntake(self):
        self.intakeSolenoid.set(DoubleSolenoid.Value.kReverse)
