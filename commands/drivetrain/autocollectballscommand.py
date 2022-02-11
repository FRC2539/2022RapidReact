from commands2 import CommandBase

import robot

from wpimath.geometry import Translation2d, Rotation2d, Pose2d
from wpimath.kinematics import ChassisSpeeds
import constants
import math


class AutoCollectBallsCommand(CommandBase):
    def __init__(self):
        super().__init__()
        # self.addRequirements(robot.drivetrain)

        # Store the current screen resolution.
        # self.resolutionX = screenResolution[0]
        # self.resolutionY = screenResolution[1]

    # def initialize(self):

    def execute(self):
        # Update the drivetrain to have it match the desired rotational velocity (omega)
        # Note: the desired rotational velocity comes from the pid controller (CHANGE)

        # Adjust the  the robot angle.
        # robot.drivetrain.setChassisSpeeds(
        #     ChassisSpeeds(0, 0, self.pidController.calculate(self.estimateBallPose()))
        # )
        print("target x:" + str(robot.ml.getX()))

    # def isFinished(self):
    #     return (
    #         self.pidController.atGoal()
    #         or not robot.drivetrain.isVisionTargetDetected()  # Does not exist yet.
    #     )

    # def end(self, interrupted):
    #     robot.drivetrain.stop()

    # def calculateBallHorizontalDistance(self):
    #     """
    #     Calculates the approximate distance to the target ball.
    #     Notes:
    #     The camera constants must be correct (camera height, angle, target ball offset).
    #     """

    #     targetBallPosition = (constants.drivetrain.resolutionX / 2) + 0  # targetX

    #     return constants.limelight.heightOffset / math.tan(totalAngle)
