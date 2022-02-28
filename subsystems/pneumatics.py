from .cougarsystem import CougarSystem
import ports
import math

from wpilib import Compressor, PneumaticsModuleType, DoubleSolenoid


class Pneumatics(CougarSystem):
    """Controls the robot compressor."""

    def __init__(self):
        super().__init__("Pneumatics")

        pneumaticsModuleType = PneumaticsModuleType.REVPH

        # Create the controller for the compressor
        self.compressor = Compressor(
            ports.pneumatics.pcmID, pneumaticsModuleType
        )

        self.compressor.enableDigital()

        # Create the controller for the intake solenoid.
        self.intakeSolenoid = DoubleSolenoid(
            ports.pneumatics.pcmID,
            pneumaticsModuleType,
            ports.intake.forwardChannel,
            ports.intake.reverseChannel
        )
        
        # Create the controller for the climber solenoid
        self.climberSolenoid = DoubleSolenoid(
            ports.pneumatics.pcmID,
            pneumaticsModuleType,
            ports.climber.forwardChannel,
            ports.climber.reverseChannel
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

    # Below was done by Ben

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