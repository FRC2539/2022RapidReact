from .cougarsystem import *

# from wpilib import AnalogInput

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

        # The sensor for telling if a ball is present.
        # self.ballSensor = AnalogInput(ports.ballsystem.sensorPort)

        # Constantly updates the ballsystem's status.
        self.constantlyUpdate(
            "Conveyor Running", lambda: self.conveyorMotor.getMotorOutputPercent() != 0
        )
        # self.constantlyUpdate(
        #     "Chamber Running", lambda: self.chamberMotor.getMotorOutputPercent() != 0
        # )

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

    # def isBallPresent(self):
    #     """
    #     Does the ball sensor see a ball?
    #     """
    #     return self.ballSensor.getValue() < 50
