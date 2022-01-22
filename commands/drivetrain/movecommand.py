from .pointfollowcommand import PointFollowCommand

from wpimath.geometry import Pose2d, Translation2d, Rotation2d


class MoveCommand(PointFollowCommand):
    """
    Moves the robot to a position x meters forward/backward.
    """

    def __init__(self, x, hiddenPoints=1):
        points = [Pose2d()]

        # Generate points along the straight line
        for i in range(hiddenPoints):
            # Calculate the next position along the line
            # Ex: 2 hidden points, _x = (1/3, then 2/3) * x
            _x = x * ((i + 1) / (hiddenPoints + 1))

            # Add the next point to the path
            points.append(Pose2d(Translation2d(_x, 0), Rotation2d(0)))

        # Add the final point to the path
        points.append(Pose2d(Translation2d(x, 0), Rotation2d(0)))

        super().__init__(points)
