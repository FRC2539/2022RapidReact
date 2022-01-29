from .cougarsystem import CougarSystem
import ports
import math
import constants
from wpilib import Compressor, DoubleSolenoid, PneumaticsModuleType
from ctre import WPI_TalonFX, NeutralMode, FeedbackDevice


class Climber(CougarSystem):
    """Controls the hardware of the climber mechanism."""

    def __init__(self):
        super().__init__("Climber")

        pneumaticsModuleType = PneumaticsModuleType.CTREPCM

        # Create the controller for the compressor
        self.compressor = Compressor(
            ports.climberPneumatics.pcmID, pneumaticsModuleType
        )

        self.compressor.enableDigital()

        # Create the controller for the solenoid
        self.climberSolenoid = DoubleSolenoid(
            ports.climberPneumatics.pcmID,
            pneumaticsModuleType,
            ports.climberPneumatics.forwardChannel,
            ports.climberPneumatics.reverseChannel,
        )

        # Create the controller for the climber motor
        self.climberMotor = WPI_TalonFX(ports.climber.motorID)

        # Configure the climber motor
        self.climberMotor.setNeutralMode(NeutralMode.Brake)
        self.climberMotor.setSafetyEnabled(False)
        self.climberMotor.setInverted(False)
        self.climberMotor.configSelectedFeedbackSensor(
            FeedbackDevice.IntegratedSensor, 0, 0
        )
        self.climberMotor.setSelectedSensorPosition(
            0
        )  # Start at zero so we don't risk over-driving downwards.

        # Standard speed of the climber, up and down.
        self.speed = constants.climber.speed

        # Climber limits.
        self.upperLimit = constants.climber.upperLimit
        self.lowerLimit = constants.climber.lowerLimit

        # Send the climber position to networktables
        self.constantlyUpdate("Climber Position", self.getPosition)

    def periodic(self):
        """
        Loops when nothing else is running in
        this subsystem. Do not call this!
        """
        self.feed()

    def raiseClimber(self):
        """
        Raises the climber using the climber motor.
        """
        if not self.atUpperLimit():
            self.climberMotor.set(self.speed)
        else:
            self.stopClimber()

    def lowerClimber(self):
        """
        Lowers the climber using the climber motor.
        """
        if not self.atLowerLimit():
            self.climberMotor.set(-self.speed)
        else:
            self.stopClimber()

    def stopClimber(self):
        """
        Stops the climber motor.
        """
        self.climberMotor.stopMotor()

    def atUpperLimit(self):
        """
        Returns true if the integrated encoder says we have
        reached our max height limit.
        """
        return self.getPosition() >= self.upperLimit

    def atLowerLimit(self):
        """
        Returns true if the integrated encoder says we have
        reached our lower limit (if the climber is lowered
        all the way; ideally, we shouldn't need this).
        """
        return self.getPosition() <= self.lowerLimit

    def abovePosition(self, position):
        """
        Determines if the climber is above the target position.
        """
        return self.getPosition() >= position

    def belowPosition(self, position):
        """
        Determines if the climber is below the target position.
        """
        return self.getPosition() <= position

    def isValidClimberPosition(self, position):
        """
        Checks if the given position is a valid climber position
        """
        return self.lowerLimit <= position <= self.upperLimit

    def getPosition(self):
        """
        Gets the current position reading from the encoder.
        """
        return self.climberMotor.getSelectedSensorPosition()

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
