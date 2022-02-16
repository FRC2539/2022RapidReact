from commands2 import SequentialCommandGroup

from commands.drivetrain.zerogyrocommand import ZeroGyroCommand
from commands.drivetrain.resetposeestimatecommand import ResetPoseEstimateCommand

from wpimath.geometry import Pose2d, Translation2d, Rotation2d


class ResetAutoStateCommand(SequentialCommandGroup):
    """
    Inform the robot of the starting configuration for the given auto.

    Parameters:
        x - an x value in meters (positive is away from the driver station)
        y - a strafe value in meters (left is positive)
        angle - an angle in radians
                (zero is straight forward from the driver station, CCW positive)
    """

    def __init__(self, x=0, y=0, angle=0):
        super().__init__()

        # Convert the parameters to a Pose2d object
        initialPose = Pose2d(Translation2d(x, y), Rotation2d(angle))

        self.addCommands(ZeroGyroCommand(angle), ResetPoseEstimateCommand(initialPose))
