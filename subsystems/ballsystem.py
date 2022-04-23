from .cougarsystem import *

from wpilib import AnalogInput, DriverStation

import wpilib

from rev import ColorSensorV3

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
        # self.bindVariable("conveyorSpeed", "Conveyor Speed", 1)
        # self.bindVariable("chamberSpeed", "Chamber Speed", 1)

        self.conveyorSpeed = 0.9
        self.chamberSpeed = 1

        # self.bindVariable("intakeConveyorSpeed", "Conveyor Intake", 1)
        # self.bindVariable("intakeChamberSpeed", "Chamber Intake", 0.7)

        self.intakeConveyorSpeed = 1
        self.intakeChamberSpeed = 0.7

        # Initialize the sensor that detects the presence of balls in the conveyor area
        self.conveyorSensor = AnalogInput(ports.ballsystem.conveyorSensor)

        # Initialize the color sensor in the chamber
        self.chamberSensor = ColorSensorV3(wpilib.I2C.Port.kOnboard)
        self.chamberProximitySensor = AnalogInput(ports.ballsystem.chamberSensor)

        self.chamberSensor.configureColorSensor(
            ColorSensorV3.ColorResolution.k18bit,
            ColorSensorV3.ColorMeasurementRate.k25ms,
        )

        # Set a threshold for the conveyor sensor
        self.analogSensorThreshold = 50

        # Set a threshold for the chamber sensor
        self.chamberSensorThreshold = 102  # 0 to 2047 (higher is closer)

        self.constantlyUpdate("Conveyor Ball", lambda: self.isConveyorBallPresent())
        self.constantlyUpdate("Chamber Ball", lambda: self.isChamberBallPresent())

        self.manualBallIntake = False

    def enableManualBallIntake(self):
        self.manualBallIntake = True

    def disableManualBallIntake(self):
        self.manualBallIntake = False

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

    def forwardConveyorIntake(self):
        """
        Run the conveyor so the balls move
        forwards.
        """
        self.moveConveyor(self.intakeConveyorSpeed)

    def forwardChamberIntake(self):
        """
        Run the chamber so the balls move
        forwards.
        """
        self.moveChamber(self.intakeChamberSpeed)

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
        return self.conveyorSensor.getValue() < self.analogSensorThreshold

    def isChamberBallPresent(self):
        """
        Returns if the sensor in the chamber sees a ball
        """
        return self.chamberProximitySensor.getValue() < self.analogSensorThreshold

    def getChamberBallColorRaw(self):
        """
        Returns the raw color read by the chamber ball sensor
        """
        return self.chamberSensor.getColor()

    def getChamberBallColor(self):
        """
        Returns the color read by the chamber ball sensor
        """
        color = self.getChamberBallColorRaw()

        # Returns the ball color
        #   Invalid if no ball
        #   Ball color otherwise
        if not self.isChamberBallPresent():
            return DriverStation.Alliance.kInvalid
        elif color.blue > color.red:
            return DriverStation.Alliance.kBlue
        else:
            return DriverStation.Alliance.kRed

    def getAllianceColor(self):
        """
        Returns the current alliance color

        (Return value is a DriverStation.Alliance (kRed, kBlue, kInvalid))
        """
        return DriverStation.getAlliance()

    def getAllianceColorRaw(self):
        """
        Returns the current alliance color

        (Return value is a DriverStation.Alliance (kRed, kBlue, kInvalid))
        """
        alliance = DriverStation.getAlliance()

        if alliance == DriverStation.Alliance.kBlue:
            return "blue"
        elif alliance == DriverStation.Alliance.kRed:
            return "red"
        else:
            return "none"
