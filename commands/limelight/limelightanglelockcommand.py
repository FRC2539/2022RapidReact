from commands2 import CommandBase

import robot
import constants
import math

from wpimath.kinematics import ChassisSpeeds
from wpimath.geometry import Pose2d


class LimelightAngleLockCommand(CommandBase):
    """
    Use the limelight to orient the robot towards the target.
    """

    def __init__(self):
        super().__init__()
        self.addRequirements(robot.drivetrain)

    def execute(self):
        # Update the drivetrain to have it match the desired rotational velocity (omega)
        robot.drivetrain.move(0, 0, robot.limelight.calculateTurnPercent())

    def end(self, interrupted):
        robot.drivetrain.stop()
