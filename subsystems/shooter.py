from .cougarsystem import *

from ctre import WPI_TalonFX, FeedbackDevice, ControlMode, NeutralMode

from networktables import NetworkTables as nt

import ports
import constants


class Shooter(CougarSystem):
    """Controls the robot's shooter."""

    def __init__(self):
        super().__init__("Shooter")

        # Initialize the first motor.
        self.shooterMotorOne = WPI_TalonFX(ports.shooter.motorOneID, "CANivore")
        self.shooterMotorOne.configSelectedFeedbackSensor(
            FeedbackDevice.IntegratedSensor, 0, 0
        )

        # Initialize the second motor.
        self.shooterMotorTwo = WPI_TalonFX(ports.shooter.motorTwoID, "CANivore")
        self.shooterMotorTwo.configSelectedFeedbackSensor(
            FeedbackDevice.IntegratedSensor, 0, 0
        )

        # Set the behavior for when both motors are in "neutral", or are not being moved.
        self.shooterMotorOne.setNeutralMode(NeutralMode.Coast)
        self.shooterMotorTwo.setNeutralMode(NeutralMode.Coast)

        # Set the PID configuration.
        self.shooterMotorOne.config_kF(0, 0.05, 0)  # Ben, no FF! -Ben
        self.shooterMotorOne.config_kP(0, 0.13, 0)
        self.shooterMotorOne.config_kI(0, 0, 0)
        self.shooterMotorOne.config_kD(0, 0.05, 0)
        self.shooterMotorOne.config_IntegralZone(0, 0, 0)

        # Set the PID configuration.
        self.shooterMotorTwo.config_kF(0, 0.05, 0)  # Ben, no FF! -Ben
        self.shooterMotorTwo.config_kP(0, 0.13, 0)
        self.shooterMotorTwo.config_kI(0, 0, 0)
        self.shooterMotorTwo.config_kD(0, 0.05, 0)
        self.shooterMotorTwo.config_IntegralZone(0, 0, 0)

        # Set the second motor to move in the opposite direction of the first.
        self.shooterMotorTwo.setInverted(True)

        # Set the range of velocities.
        self.maxVel = 3000
        self.minVel = 100

        # Reject ball velocity
        self.rejectRPM1 = 800
        self.rejectRPM2 = 1000

        # Low goal rpms
        # self.lowGoalRPM1 = 1150
        # self.lowGoalRPM2 = 900
        self.lowGoalRPM1 = 1150
        self.lowGoalRPM2 = 900

        # High goal rpms
        # self.highGoalRPM1 = 900
        # self.highGoalRPM2 = 2300

        # self.highGoalRPM1 = 1000
        # self.highGoalRPM2 = 2500
        self.bindVariable("highGoalRPM1", "highGoalRPM1", 980)
        self.bindVariable("highGoalRPM2", "highGoalRPM2", 2480)

        # High goal slightly behind line
        # self.behindLineRPM1 = 2600
        # self.behindLineRPM2 = 900

        self.bindVariable("behindLineRPM1", "behindLineRPM1", 950)
        self.bindVariable("behindLineRPM2", "behindLineRPM2", 2950)

        self.behindLineAngle = 24

        self.bindVariable("testRPM1", "Test RPM 1", 2500)
        self.bindVariable("testRPM2", "Test RPM 2", 1500)

        # Configuration for limelight shooting
        self.bindVariable("startRPM1", "Start RPM1", 2600)  # 2650
        self.bindVariable("startRPM2", "Start RPM2", 1075)

        # 3200, 1200

        # Close RPMS
        # 2700, 1700

        # 3000, 850, 0.8
        # 930, 850 mult

        self.bindVariable(
            "rpm1Multiplier", "RPM1 Multiplier", 800
        )  # 350 was working well
        self.bindVariable("rpm2Multiplier", "RPM2 Multiplier", 500)
        self.bindVariable("startDistance", "Start Distance", 0.8)

        # Constantly updates the hood's status.
        self.constantlyUpdate(
            "Shooter Running", lambda: self.shooterMotorOne.getMotorOutputPercent() != 0
        )
        self.constantlyUpdate("Shooter RPM", lambda: float(self.getRPM1()))
        self.constantlyUpdate("Shooter RPM2", lambda: float(self.getRPM2()))

    def periodic(self):
        """
        Loops when nothing else is running in
        this subsystem. Do not call this!
        """
        self.feed()

    def setRPM(self, rpm1, rpm2):
        """
        Sets the two shooting motors to given RPMs.
        """
        self.shooterMotorOne.set(ControlMode.Velocity, self.rpmToSensor(rpm1))
        self.shooterMotorTwo.set(ControlMode.Velocity, self.rpmToSensor(rpm2))

    def setPercent(self, val1, val2):
        """
        Sets the two shooter motors to
        given percent outputs.
        """
        self.shooterMotorOne.set(ControlMode.PercentOutput, val1)
        self.shooterMotorTwo.set(ControlMode.PercentOutput, val2)

    def stopShooter(self):
        """
        Stops both shooter motors.
        """
        self.shooterMotorOne.stopMotor()
        self.shooterMotorTwo.stopMotor()

    def rpmToSensor(self, rpm):
        return self.rpmToSensorRaw(rpm * constants.shooter.gearRatio)

    def sensorToRPM(self, units):
        return self.sensorToRPMRaw(units) / constants.shooter.gearRatio

    def rpmToSensorRaw(self, rpm):
        """
        Convert a standard output RPM into
        a tick units for the robot. Please note,
        this is before the gear ratio on the
        shooter, which is not a 1:1.
        """
        return (rpm * 2048) / 600

    def sensorToRPMRaw(self, units):
        """
        Convert a value in tick units into
        a human-comprehendible RPM. Please note,
        this is before the gear ratio on the
        shooter, which is not a 1:1.
        """
        return (units * 600) / 2048

    def getRPM1(self):
        """
        Returns the current RPM of motorOne.
        """
        return self.sensorToRPM(self.shooterMotorOne.getSelectedSensorVelocity())

    def getRPM2(self):
        """
        Returns the current RPM of motorTwo.
        """
        return self.sensorToRPM(self.shooterMotorTwo.getSelectedSensorVelocity())

    def calculateRPMsFromDistance(self, distance):
        """
        Uses the distance from the target to calculate the shooter speeds.
        """
        rpm1 = self.startRPM1 + self.rpm1Multiplier * distance
        rpm2 = self.startRPM2 + self.rpm2Multiplier * distance

        return [rpm1, rpm2]
