from threading import currentThread
from commands2 import CommandBase
import constants
import math

import robot

from wpimath.trajectory import TrapezoidProfileRadians

from wpimath.kinematics import ChassisSpeeds
from wpimath.geometry import Pose2d, Rotation2d, Translation2d


class TurnToCommand(CommandBase):
    """
    Turns a certain number of radians, or to a specified angle.

    turnAngle - CCW Positive, radians
    """

    def __init__(
        self,
        turnAngle,
        relative=True,
        turnSpeed=1,
        accelerationRate=4,
        minRotationSpeed=constants.drivetrain.angularSpeedMinimum,
    ):
        super().__init__()

        self.addRequirements(robot.drivetrain)

        self.relative = relative

        # (-) accounts for the fact that the robot is counterclockwise positive
        self.turnAngle = -turnAngle

        # sets how close the robot must be to the target angle to stop the command
        self.radianTolerance = constants.drivetrain.autoTolerance.rotation().radians()

        # sets the tolerances for the robot
        self.minRotationSpeed = minRotationSpeed
        self.constraints = TrapezoidProfileRadians.Constraints(
            turnSpeed, accelerationRate
        )

        self.pidController = robot.drivetrain.driveThetaController

    def initialize(self):
        currentPosition = self.getMeasurement()
        self.pidController.reset(currentPosition)

        # takes account for if the command is relative or not
        if self.relative:
            self.pidController.setGoal(self.turnAngle + currentPosition)
        else:
            self.pidController.setGoal(self.turnAngle)
        self.pidController.setTolerance(self.radianTolerance)

        # sets the contraints from earlier
        # self.pidController.setConstraints(self.constraints)

        # sets the PID values for the pidController
        self.pidController.setPID(1, 0, 0)

    def execute(self):
        # sets the current chassis speeds based on the
        robot.drivetrain.setChassisSpeeds(
            ChassisSpeeds(0, 0, self.pidController.calculate(self.getMeasurement()))
        )

    def isFinished(self):
        # checks if the current angle of the robot is within the tolerance of the wanted angle
        return self.pidController.atGoal()

    def end(self, interrupted):
        # stops the robot if the command is for some reason halted
        robot.drivetrain.stop()

    def getMeasurement(self):
        return robot.drivetrain.navX.getAngle() / 180 * math.pi
