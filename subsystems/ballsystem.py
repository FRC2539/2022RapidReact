from .cougarsystem import *

from wpilib import AnalogInput

import wpilib

from rev.color import ColorSensorV3

import ports

from ctre import WPI_TalonSRX, NeutralMode, ControlMode


class BallSystem(CougarSystem):
    """
    Controls the balls within the robot.

    This includes the angled conveyor, and the vertical chamber.
    """

    def __init__(self):
        super().__init__("BallSystem")

        self.conveyorMotor = WPI_TalonSRX(ports.ballsystem.conveyorMotorID)
        self.chamberMotor = WPI_TalonSRX(ports.ballsystem.chamberMotorID)

        self.configureMotor(self.conveyorMotor)
        self.configureMotor(self.chamberMotor)

        # INFO: Percentages are from 0 - 1, 1 being 100%
        self.bindVariable("conveyorSpeed", "Conveyor Speed", 1.0)
        self.bindVariable("chamberSpeed", "Chamber Speed", 1.0)

        # Initialize the sensor that detects the presence of balls in the conveyor area
        self.conveyorSensor = AnalogInput(ports.ballsystem.sensorPort)

        # Initialize the color sensor in the chamber
        self.chamberSensor = ColorSensorV3(wpilib.I2C.Port.kOnboard)

        self.colorSensor.configureColorSensor(
            ColorSensorV3.ColorResolution.k18bit,
            ColorSensorV3.ColorMeasurementRate.k50ms,
        )

        # Set a threshold for the conveyor sensor
        self.conveyorSensorThreshold = 50

        # Constantly updates the ballsystem's status.
        self.constantlyUpdate(
            "Conveyor Running", lambda: self.conveyorMotor.getMotorOutputPercent() != 0
        )
        self.constantlyUpdate(
            "Chamber Running", lambda: self.chamberMotor.getMotorOutputPercent() != 0
        )

    def configureMotor(self, motor):
        motor.setNeutralMode(NeutralMode.Brake)
        motor.setInverted(True)

    def periodic(self):
        """
        Loops when nothing else is running in
        this subsystem. Do not call this!
        """
        self.feed()

    def forwardAll(self):
        self.moveConveyor(self.conveyorSpeed)
        self.moveChamber(self.chamberSpeed)

    def backwardAll(self):
        self.moveConveyor(-self.conveyorSpeed)
        self.moveChamber(-self.chamberSpeed)

    def forwardConveyor(self):
        """
        Run the conveyor so the balls move
        forwards.
        """
        self.moveConveyor(self.conveyorSpeed)

    def forwardChamber(self):
        """
        Run the chamber so the balls move
        forwards.
        """
        self.moveChamber(self.chamberSpeed)

    def backwardConveyor(self):
        """
        Reverse the conveyor so the balls
        move backwards.
        """
        self.moveConveyor(-self.conveyorSpeed)

    def backwardChamber(self):
        """
        Reverse the chamber so the balls
        move backwards.
        """
        self.moveChamber(-self.chamberSpeed)

    def moveConveyor(self, speed):
        """
        Basic move method to set custom speed to the motor.
        """
        self.conveyorMotor.set(ControlMode.PercentOutput, speed)

    def moveChamber(self, speed):
        """
        Basic move method to set custom speed to the motor.
        """
        self.chamberMotor.set(ControlMode.PercentOutput, speed)

    def stopConveyor(self):
        """
        Stops the conveyor motor.
        """
        self.conveyorMotor.stopMotor()

    def stopChamber(self):
        """
        Stops the chamber motor.
        """
        self.chamberMotor.stopMotor()

    def isConveyorBallPresent(self):
        """
        Returns if the sensor in the conveyor sees a ball
        """
        return self.conveyorSensor.getValue() < self.conveyorSensorThreshold
