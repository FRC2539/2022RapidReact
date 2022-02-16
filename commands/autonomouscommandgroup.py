from wpilib import DriverStation

from commands2 import SequentialCommandGroup

from networktables import NetworkTables

import math
import robot, constants

from commands import autoconfig

from commands.drivetrain.movecommand import MoveCommand
from commands.drivetrain.custommovecommand import CustomMoveCommand
from commands.drivetrain.resetautostatecommand import ResetAutoStateCommand
from commands.drivetrain.turncommand import TurnCommand
from commands.drivetrain.turninplacecommand import TurnInPlaceCommand
from commands.drivetrain.pointfollowcommand import PointFollowCommand
from commands.drivetrain.trajectoryfollowercommand import TrajectoryFollowerCommand

from commands.shooter.surrogateshootercommand import SurrogateShooterCommand

from wpimath.geometry import Pose2d


class AutonomousCommandGroup(SequentialCommandGroup):
    """Note: add a 0 at the end of the auto name to set it as default."""

    def __init__(self):
        super().__init__()

        ds = DriverStation.getInstance()
        self.msg = ds.getGameSpecificMessage()

        self.currentAuto = autoconfig.getAutoProgram()
        toRun = self.currentAuto

        for var in dir(self):  # Identifies the method to setup.
            if var.lower() == self.currentAuto:
                toRun = var
                break

        eval("self." + toRun + "()")  # Runs the method

    def moveTest0(self):
        self.addCommands(
            ResetAutoStateCommand(x=0, y=0, angle=0),
            CustomMoveCommand(x=0.5, y=0.5, relative=False),
            TurnCommand(3.14 / 2, relative=False),
            # TurnCommand(0, relative=False),
            # TurnCommand(-3.14 / 2, relative=False),
            CustomMoveCommand(x=-0.5, y=-0.5, relative=False),
            # CustomMoveCommand(x=1, relative=False),
            # TurnCommand(3.14 / 2, relative=False),
            # TurnCommand(-3.14 / 2, relative=False),
            # CustomMoveCommand(x=0, relative=False),
            # TurnCommand(6.28, relative=False),
        )

    def trajectory(self):
        self.addCommands(
            ResetAutoStateCommand(x=0, y=0, angle=0),
            TrajectoryFollowerCommand(robot.drivetrain.trajectory),
        )

    def turnInPlaceTest(self):
        self.addCommands(
            ResetAutoStateCommand(x=0, y=0, angle=0),
            TurnInPlaceCommand(6.28 * 2, accelerationRate=4, turnSpeed=1),
            # MoveCommand(1, linearVelocity=0.5, matchHeading=False),
            TurnInPlaceCommand(6.28, accelerationRate=4, turnSpeed=1),
        )

    def fourBall(self):
        """Collect the leftmost red ball, shoot 2, then collect 2 more balls and shoot those."""
        self.addCommands(
            ResetAutoStateCommand(x=0, y=0, angle=0),
            CustomMoveCommand(x=0, y=1.89),
            # Intake ball command
            CustomMoveCommand(x=0, y=-1.89),
            TurnCommand(turnAngle=-1.46),
            CustomMoveCommand(x=6.67, y=0.71),
            CustomMoveCommand(x=-2.69, y=0.29),
        )

    def twoBallLeftND(self):
        """Collects the leftmost red ball and shoots both. ND stands for Non-Disruptive (no blue ball interference)."""
        self.addCommands(
            ResetAutoStateCommand(x=0, y=0, angle=0),
            CustomMoveCommand(x=0, y=1.89),
            # Intake ball command
        )

    def twoBallMidND(self):
        """Collects the middle red ball and shoots both. ND stands for Non-Disruptive (no blue ball interference)."""
        self.addCommands(
            ResetAutoStateCommand(x=0, y=0, angle=0),
            CustomMoveCommand(x=1.23, y=0.6),
            # ShootCommand
        )

    def interrupted(self):
        pass
