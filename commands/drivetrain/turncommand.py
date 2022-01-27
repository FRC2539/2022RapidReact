from .pointfollowcommand import PointFollowCommand

from wpimath.geometry import Pose2d, Translation2d, Rotation2d


class TurnCommand(PointFollowCommand):
    """
    Rotates the robot by an angle in degrees.
    """

    def __init__(self, degrees, hiddenPoints=3):
        points = [Pose2d()]

        # Generate points along a rotation
        for i in range(hiddenPoints):
            # Calculate the next position along the rotation
            # Ex: 2 hidden points, _theta = (1/3, then 2/3) * degrees
            _theta = degrees * ((i + 1) / (hiddenPoints + 1))

            # Add the next point to the path
            points.append(Pose2d(Translation2d(0, 0), Rotation2d.fromDegrees(_theta)))

        # Add the final point to the path
        points.append(Pose2d(Translation2d(0, 0), Rotation2d.fromDegrees(degrees)))

        super().__init__(points, matchHeading=True)
