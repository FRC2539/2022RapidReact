from wpilib import DriverStation

from commands2 import (
    ParallelRaceGroup,
    SequentialCommandGroup,
    ParallelCommandGroup,
    WaitCommand,
)

from networktables import NetworkTables

import math
import robot, constants

from commands import autoconfig

from commands.drivetrain.movecommand import MoveCommand
from commands.drivetrain.custommovecommand import CustomMoveCommand
from commands.drivetrain.resetautostatecommand import ResetAutoStateCommand
from commands.drivetrain.turncommand import TurnCommand
from commands.drivetrain.turninplacecommand import TurnInPlaceCommand
from commands.drivetrain.turntocommand import TurnToCommand
from commands.drivetrain.pointfollowcommand import PointFollowCommand
from commands.drivetrain.moveforwardcommand import MoveForwardCommand
from commands.drivetrain.trajectoryfollowercommand import TrajectoryFollowerCommand

from commands.shooter.surrogateshootercommand import SurrogateShooterCommand
from commands.shooter.highgoalshootcommand import HighGoalShootCommand
from commands.shooter.highgoalfendercommand import HighGoalFenderCommand

from commands.intake.intakecommand import IntakeCommand
from commands.intake.rejectcommand import RejectCommand

from commands.ballsystem.forwardballsystemcommand import ForwardBallSystemCommand


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

    def oldthreeBall(self):
        self.addCommands(
            ResetAutoStateCommand(angle=-math.pi / 2),
            # AimAndShootCommand(),
            TurnCommand(-math.pi / 2, relative=False),
            # StartIntakeCommand(),
            MoveForwardCommand(1.7),
            MoveForwardCommand(-0.79),
            TurnCommand(-1.83),
            MoveForwardCommand(2.54),
            # StopIntakeCommand(),
            TurnCommand(0.79),
            # AimAndShootCommand(),
        )

    def threeBall0(self):
        """
        Currently following the correct path, but sometimes doesn't stop on the long
        forward path.
        """
        self.addCommands(
            ResetAutoStateCommand(angle=-math.radians(78)),
            # AimAndShootCommand(),
            ParallelRaceGroup(HighGoalFenderCommand(), WaitCommand(3)),
            TurnCommand(-math.radians(78), relative=False),
            # StartIntakeCommand(),
            ParallelRaceGroup(
                SequentialCommandGroup(
                    MoveForwardCommand(1.09),
                    MoveForwardCommand(-0.79),
                    TurnCommand(-1.48),
                    MoveForwardCommand(2.54),
                ),
                ForwardBallSystemCommand(),
            ),
            # StopIntakeCommand(),
            TurnCommand(0.79),
            # AimAndShootCommand(),
            HighGoalShootCommand(),
        )

    def matthewsMoveCommand(self):
        self.addCommands(
            ResetAutoStateCommand(angle=math.pi / 2),
            TurnCommand(0, relative=False),
            MoveForwardCommand(2),
            TurnCommand(math.pi / 4),
            MoveForwardCommand(-math.sqrt(2)),
            TurnCommand(-math.pi / 2),
            MoveForwardCommand(math.sqrt(2)),
            TurnCommand(0, relative=False),
            MoveForwardCommand(-2),
            TurnCommand(-math.pi / 3 * 2, relative=False),
        )

    def moveTest(self):
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

    def turnToTest(self):
        self.addCommands(
            ResetAutoStateCommand(),
            TurnToCommand(math.pi / 2),
            TurnToCommand(-math.pi),
            TurnToCommand(math.pi / 2),
        )

    # def fiveBall(self):
    # """Immediately shoots a red ball, collects 2 balls and shoots them, then collects a red ball + one from the human player station."""
    # self.addCommands(
    # ResetAutoStateCommand(x=0, y=0, angle=-1.57), # -90 degrees
    # SurrogateShooterCommand(),
    # ParallelCommandGroup(
    # IntakeCommand(),
    # CustomMoveCommand(x=0, y=1.89),
    # )
    # TurnCommand(-1.97), # -113 degrees
    # CustomMoveCommand(x=2.52, y=-1.07),
    # TurnCommand(
    # )

    def fourBall(self):
        """Collect the leftmost red ball, shoot 2, then collect 2 more balls and shoot those."""
        self.addCommands(
            # Collect the leftmost ball.
            ResetAutoStateCommand(x=0, y=0, angle=-1.57),  # -90 degrees.
            ParallelCommandGroup(
                IntakeCommand(),
                CustomMoveCommand(x=0, y=1.89, relative=False),
            ),
            # Collect the remaining two balls.
            CustomMoveCommand(x=0, y=0, relative=False),
            TurnCommand(turnAngle=-1.46),
            ParallelCommandGroup(
                IntakeCommand(),
                CustomMoveCommand(x=6.67, y=0.71, relative=False),
            ),
            # Return to shoot.
            CustomMoveCommand(x=-2.69, y=0.29),
            TurnCommand(0.35),  # 20 degrees
            SurrogateShooterCommand(),
        )

    def twoBallLeftND(self):
        """Collects the leftmost red ball and shoots both. ND standss for Non-Disruptive (no blue ball interference)."""
        self.addCommands(
            ResetAutoStateCommand(x=0, y=0, angle=-1.57),  # -90 degrees
            ParallelCommandGroup(
                IntakeCommand(),
                CustomMoveCommand(x=0, y=1.89),
            ),
            SurrogateShooterCommand(),
        )

    def twoBallMidND(self):
        """Collects the middle red ball and shoots both. ND stands for Non-Disruptive (no blue ball interference)."""
        self.addCommands(
            ResetAutoStateCommand(x=0, y=0, angle=-2.15),  # -123 degrees
            ParallelCommandGroup(
                IntakeCommand(),
                CustomMoveCommand(x=1.23, y=0.6),
            ),
            TurnCommand(0.54),  # 31 degrees
            SurrogateShooterCommand(),
        )

    def twoBallRightND(self):
        """Collects the rightmost red ball and shoots both. ND stands for Non-Disruptive (no blue ball interference)"""
        self.addCommands(
            ResetAutoStateCommand(x=0, y=0, angle=2.30),  # 132 degrees
            ParallelCommandGroup(
                IntakeCommand(),
                CustomMoveCommand(x=0.75, y=-0.83),
            ),
            TurnCommand(0.17),  # 10 degrees
            SurrogateShooterCommand(),
        )

    def twoBallLeftYD(self):
        """Collects the rightmost blue ball and yeets it away from the centerline before scoring 1 red ball. YD stands for Yes-Disruptive (blue ball interference)."""
        self.addCommands(
            # Collect and yeet the blue ball.
            ResetAutoStateCommand(x=0, y=0, angle=-1.27),  # -73 degrees
            ParallelCommandGroup(
                IntakeCommand(),
                CustomMoveCommand(x=-0.35, y=1.16),
            ),
            TurnCommand(-2.04),  # -117 degrees
            RejectCommand(),  # Could potentially be switched with shoot for maximum yeet.
            # Shoot the red ball.
            TurnCommand(1.95),  # 112 degrees
            SurrogateShooterCommand(),
        )

    def twoBallRightYD(self):
        """Collects the leftmost blue ball and yeets it away from the centerline before scoring 1 red ball. YD stands for Yes-Disruptive (blue ball interference)."""
        self.addCommands(
            # Collect and yeet the blue ball.
            ResetAutoStateCommand(x=0, y=0, angle=-2.06),  # -118 degrees
            ParallelCommandGroup(
                IntakeCommand(),
                CustomMoveCommand(x=1.16, y=0.35),
            ),
            TurnCommand(-0.79),  # -45 degrees
            RejectCommand(),  # Could potentially be switched with shoot for maximum yeet.
            TurnCommand(0.61),  # 35 degrees
            SurrogateShooterCommand(),
        )

    def interrupted(self):
        pass
