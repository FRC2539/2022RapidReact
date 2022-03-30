from bz2 import compress
from .cougarsystem import CougarSystem
import ports
import math

from wpilib import PneumaticHub, Compressor, PneumaticsModuleType, DoubleSolenoid


class Pneumatics(CougarSystem):
    """Controls the robot compressor."""

    def __init__(self):
        super().__init__("Pneumatics")

        # Create the controller for the compressor
        self.compressor = Compressor(ports.pneumatics.pcmID, PneumaticsModuleType.REVPH)

        # Create the controller for the intake solenoid.
        self.intakeSolenoid = DoubleSolenoid(
            ports.pneumatics.pcmID,
            PneumaticsModuleType.REVPH,
            ports.intake.forwardChannel,
            ports.intake.reverseChannel,
        )

        # Create the controller for the climber solenoid
        self.climberSolenoid = DoubleSolenoid(
            ports.pneumatics.pcmID,
            PneumaticsModuleType.REVPH,
            ports.climber.forwardChannel,
            ports.climber.reverseChannel,
        )

        # Create the controller for the hood solenoid
        self.hoodSolenoid = DoubleSolenoid(
            ports.pneumatics.pcmID,
            PneumaticsModuleType.REVPH,
            ports.hood.forwardChannel,
            ports.hood.reverseChannel,
        )

    def periodic(self):
        """
        Loops when nothing else is running in
        this subsystem. Do not call this!
        """
        self.feed()

        if self.compressor.getPressureSwitchValue() and not self.compressor.enabled():
            self.compressor.start()
        elif self.compressor.getPressureSwitchValue():
            self.compressor.start()
        else:
            self.compressor.stop()

    def extendIntake(self):
        self.intakeSolenoid.set(DoubleSolenoid.Value.kForward)

    def retractIntake(self):
        self.intakeSolenoid.set(DoubleSolenoid.Value.kReverse)

    def extendHood(self):
        self.hoodSolenoid.set(DoubleSolenoid.Value.kForward)

    def retractHood(self):
        self.hoodSolenoid.set(DoubleSolenoid.Value.kReverse)

    def extendClimberArm(self):
        """
        Extends the climber arm solenoid, raising it to its vertical position.
        """
        self.climberSolenoid.set(DoubleSolenoid.Value.kForward)

    def retractClimberArm(self):
        """
        Retracts the climber arm solenoid, lowering it to its angled position.
        """
        self.climberSolenoid.set(DoubleSolenoid.Value.kReverse)

    def toggleClimberArm(self):
        self.climberSolenoid.toggle()

    def isSolenoidForward(self):
        return self.climberSolenoid.get() == DoubleSolenoid.Value.kForward

    def isSolenoidOff(self):
        return self.climberSolenoid.get() == DoubleSolenoid.Value.kOff
