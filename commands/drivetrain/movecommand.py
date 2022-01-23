from .pointfollowcommand import PointFollowCommand

from wpimath.geometry import Pose2d, Translation2d, Rotation2d


class MoveCommand(PointFollowCommand):
    """
    Moves the robot to a given point, with a given heading.

    Parameters:
        x - meters forward / backward (-)
        y - meters left / right (-)
        angle - heading angle in degrees (decoupled from the direction of travel)

        linearVelocity - the desired speed of travel in m/s (default = None)
        hiddenPoints - the precision of the path (more points -> more precision)
    """

    def __init__(self, x, y=0, angle=0, linearVelocity=None, hiddenPoints=1):
        points = [Pose2d()]

        # Generate points along the straight line
        for i in range(hiddenPoints):
            # Calculate the next position along the line
            # Ex: 2 hidden points, _x = (1/3, then 2/3) * x
            pathPercentage = ((i + 1) / (hiddenPoints + 1))
            _x = x * pathPercentage
            _y = y * pathPercentage

            # Add the next point to the path
            points.append(Pose2d(Translation2d(_x, _y), Rotation2d.fromDegrees(angle)))

        # Add the final point to the path
        points.append(Pose2d(Translation2d(x, y), Rotation2d.fromDegrees(angle)))

        super().__init__(points, linearVelocity=linearVelocity, matchHeading=True)
