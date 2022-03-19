from commands2 import InstantCommand
from numpy import angle
from wpilib import DriverStation

from commands2 import (
    ParallelRaceGroup,
    SequentialCommandGroup,
    ParallelCommandGroup,
    WaitCommand,
)

from networktables import NetworkTables

import math
from commands.drivetrain.funnymovecommand import FunnyMoveCommand
from commands.intakeballscommandgroup import IntakeBallsCommandGroup
from commands.limelight.limelightanglelockcommand import LimelightAngleLockCommand
from commands.shooter.customshootcommand import CustomShootCommand
from commands.shooter.highgoallinecommand import HighGoalLineCommand
from commands.shooter.lowgoalshootcommand import LowGoalShootCommand
from commands.shooter.sethoodanglecommand import SetInitialHoodAngleCommand
from commands.shooter.setshooterrpmscommand import SetShooterRPMsCommand
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
from commands.drivetrain.bezierpathcommand import BezierPathCommand

from commands.shooter.surrogateshootercommand import SurrogateShooterCommand
from commands.shooter.highgoalshootcommand import HighGoalShootCommand
from commands.shooter.highgoalfendercommand import HighGoalFenderCommand

from commands.intake.intakecommand import IntakeCommand
from commands.intake.rejectcommand import RejectCommand

from commands.ballsystem.forwardballsystemcommand import ForwardBallSystemCommand
from commands.drivetrain.autocollectballscommandgroup import (
    AutoCollectBallsCommandGroup,
)


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

    # def threeBall(self):
    #     """
    #     Currently following the correct path, needs limelight aiming and shooting
    #     """
    #     self.addCommands(
    #         ResetAutoStateCommand(angle=-math.radians(78)),
    #         ParallelRaceGroup(SetInitialHoodAngleCommand(), WaitCommand(0.5)),
    #         ParallelRaceGroup(
    #             CustomShootCommand(rpm1=1200, rpm2=2300, hoodAngle=18), WaitCommand(3)
    #         ),
    #         TurnCommand(-math.radians(78), relative=False),
    #         ParallelRaceGroup(
    #             SequentialCommandGroup(
    #                 FunnyMoveCommand(1.09, torySlow=12000),
    #                 FunnyMoveCommand(-0.79, torySlow=12000),
    #                 TurnCommand(-1.38),
    #             ),
    #             IntakeBallsCommandGroup(),
    #         ),
    #         AutoCollectBallsCommandGroup(endOnBallPickup=True),
    #         TurnCommand(0.79),
    #         FunnyMoveCommand(-1.5),
    #         ParallelRaceGroup(
    #             HighGoalLineCommand(), LimelightAngleLockCommand(), WaitCommand(3)
    #         ),
    #     )

    def definitelyNotBensAuto(self):
        self.addCommands(
            # BezierPathCommand([[0, 0], [0, 24]], speed=0.45, stopWhenDone=True),
            TurnCommand(3.1415956535 / 2),
            # BezierPathCommand([[0, 0], [20, 80]], speed=0.8, stopWhenDone=True),
        )

    def twoBall(self):
        self.addCommands(
            ResetAutoStateCommand(angle=0),
            ParallelRaceGroup(SetInitialHoodAngleCommand(), WaitCommand(2)),
            ParallelRaceGroup(
                IntakeBallsCommandGroup(),
                SequentialCommandGroup(
                    FunnyMoveCommand(1.1),
                    WaitCommand(0.5),
                ),
            ),
            FunnyMoveCommand(-0.1),
            ParallelRaceGroup(HighGoalLineCommand(), WaitCommand(3)),
        )

    def fourBall(self):
        self.addCommands(
            ResetAutoStateCommand(angle=0),
            TrajectoryFollowerCommand(robot.drivetrain.trajectory),
        )

    def tweeBall(self):
        self.addCommands(
            ResetAutoStateCommand(angle=0),
            ParallelRaceGroup(
                IntakeBallsCommandGroup(),
                SequentialCommandGroup(
                    FunnyMoveCommand(1.1),
                    WaitCommand(0.3),
                ),
                SetInitialHoodAngleCommand(),
            ),
            FunnyMoveCommand(-0.1),
            ParallelRaceGroup(HighGoalLineCommand(), WaitCommand(3)),
            TurnCommand(3.142 / 2),
            FunnyMoveCommand(0.5),
        )

    def fourBallStraightUp(self):
        self.addCommands(
            ResetAutoStateCommand(angle=0),
            ParallelRaceGroup(
                IntakeBallsCommandGroup(),
                SequentialCommandGroup(
                    FunnyMoveCommand(1.6, angle=-15, torySlow=31000),
                ),
                SetInitialHoodAngleCommand(),
            ),
            ParallelRaceGroup(
                IntakeBallsCommandGroup(),
                HighGoalShootCommand(),
                WaitCommand(3.5),
            ),
            TurnCommand(math.radians(-24.12)),
            ParallelRaceGroup(
                IntakeBallsCommandGroup(),
                SequentialCommandGroup(
                    FunnyMoveCommand(4, torySlow=31000),
                    WaitCommand(2),
                    ParallelRaceGroup(
                        FunnyMoveCommand(-3.2, torySlow=31000),
                        SetInitialHoodAngleCommand(),
                    ),
                ),
            ),
            TurnCommand(1.042 / 2),
            ParallelRaceGroup(
                LimelightAngleLockCommand(),
                WaitCommand(0.5),
                SetInitialHoodAngleCommand(),
            ),
            ParallelRaceGroup(
                HighGoalShootCommand(),
                WaitCommand(3),
            ),
        )

    def fiveBallStraightUp0(self):
        self.addCommands(
            ResetAutoStateCommand(angle=0),
            ParallelRaceGroup(
                IntakeBallsCommandGroup(),
                SetInitialHoodAngleCommand(),
                SequentialCommandGroup(
                    FunnyMoveCommand(1.2, angle=0, torySlow=80000),
                ),
            ),
            ParallelRaceGroup(
                IntakeBallsCommandGroup(),
                HighGoalShootCommand(),
                WaitCommand(3),
            ),
            FunnyMoveCommand(2.6, angle=-125, torySlow=80000),
            TurnCommand(math.radians(-36)),
            ParallelRaceGroup(
                FunnyMoveCommand(0.6, angle=0, torySlow=80000),
                IntakeBallsCommandGroup(),
            ),
            ParallelRaceGroup(
                LimelightAngleLockCommand(),
                WaitCommand(0.5),
                SetInitialHoodAngleCommand(),
            ),
            ParallelRaceGroup(
                HighGoalShootCommand(),
                WaitCommand(1.7),
            ),
            TurnCommand(math.radians(-22.12)),
            ParallelRaceGroup(
                IntakeBallsCommandGroup(),
                SequentialCommandGroup(
                    FunnyMoveCommand(4, torySlow=80000),
                    WaitCommand(2),
                    ParallelRaceGroup(
                        FunnyMoveCommand(-3.2, torySlow=80000),
                        SetInitialHoodAngleCommand(),
                    ),
                ),
            ),
            TurnCommand(1.042 / 2),
            ParallelRaceGroup(
                LimelightAngleLockCommand(),
                WaitCommand(0.5),
                SetInitialHoodAngleCommand(),
            ),
            ParallelRaceGroup(
                HighGoalShootCommand(),
                WaitCommand(3),
            ),
        )

    # def shootTest(self):
    #     self.addCommands(
    #         ResetAutoStateCommand(angle=0),
    #         FunnyMoveCommand(1.3),
    #         ParallelRaceGroup(
    #             HighGoalLineCommand(), LimelightAngleLockCommand(), WaitCommand(2.5)
    #         ),
    #     )

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

    # def fourBall(self):
    #     """Collect the leftmost red ball, shoot 2, then collect 2 more balls and shoot those."""
    #     self.addCommands(
    #         # Collect the leftmost ball.
    #         ResetAutoStateCommand(x=0, y=0, angle=-1.57),  # -90 degrees.
    #         ParallelCommandGroup(
    #             IntakeCommand(),
    #             CustomMoveCommand(x=0, y=1.89, relative=False),
    #         ),
    #         # Collect the remaining two balls.
    #         CustomMoveCommand(x=0, y=0, relative=False),
    #         TurnCommand(turnAngle=-1.46),
    #         ParallelCommandGroup(
    #             IntakeCommand(),
    #             CustomMoveCommand(x=6.67, y=0.71, relative=False),
    #         ),
    #         # Return to shoot.
    #         CustomMoveCommand(x=-2.69, y=0.29),
    #         TurnCommand(0.35),  # 20 degrees
    #         SurrogateShooterCommand(),
    #     )

    # def twoBallLeftND(self):
    #     """Collects the leftmost red ball and shoots both. ND standss for Non-Disruptive (no blue ball interference)."""
    #     self.addCommands(
    #         ResetAutoStateCommand(x=0, y=0, angle=-1.57),  # -90 degrees
    #         ParallelCommandGroup(
    #             IntakeCommand(),
    #             CustomMoveCommand(x=0, y=1.89),
    #         ),
    #         SurrogateShooterCommand(),
    #     )

    # def twoBallMidND(self):
    #     """Collects the middle red ball and shoots both. ND stands for Non-Disruptive (no blue ball interference)."""
    #     self.addCommands(
    #         ResetAutoStateCommand(x=0, y=0, angle=-2.15),  # -123 degrees
    #         ParallelCommandGroup(
    #             IntakeCommand(),
    #             CustomMoveCommand(x=1.23, y=0.6),
    #         ),
    #         TurnCommand(0.54),  # 31 degrees
    #         SurrogateShooterCommand(),
    #     )

    # def twoBallRightND(self):
    #     """Collects the rightmost red ball and shoots both. ND stands for Non-Disruptive (no blue ball interference)"""
    #     self.addCommands(
    #         ResetAutoStateCommand(x=0, y=0, angle=2.30),  # 132 degrees
    #         ParallelCommandGroup(
    #             IntakeCommand(),
    #             CustomMoveCommand(x=0.75, y=-0.83),
    #         ),
    #         TurnCommand(0.17),  # 10 degrees
    #         SurrogateShooterCommand(),
    #     )

    # def twoBallLeftYD(self):
    #     """Collects the rightmost blue ball and yeets it away from the centerline before scoring 1 red ball. YD stands for Yes-Disruptive (blue ball interference)."""
    #     self.addCommands(
    #         # Collect and yeet the blue ball.
    #         ResetAutoStateCommand(x=0, y=0, angle=-1.27),  # -73 degrees
    #         ParallelCommandGroup(
    #             IntakeCommand(),
    #             CustomMoveCommand(x=-0.35, y=1.16),
    #         ),
    #         TurnCommand(-2.04),  # -117 degrees
    #         RejectCommand(),  # Could potentially be switched with shoot for maximum yeet.
    #         # Shoot the red ball.
    #         TurnCommand(1.95),  # 112 degrees
    #         SurrogateShooterCommand(),
    #     )

    # def twoBallRightYD(self):
    #     """Collects the leftmost blue ball and yeets it away from the centerline before scoring 1 red ball. YD stands for Yes-Disruptive (blue ball interference)."""
    #     self.addCommands(
    #         # Collect and yeet the blue ball.
    #         ResetAutoStateCommand(x=0, y=0, angle=-2.06),  # -118 degrees
    #         ParallelCommandGroup(
    #             IntakeCommand(),
    #             CustomMoveCommand(x=1.16, y=0.35),
    #         ),
    #         TurnCommand(-0.79),  # -45 degrees
    #         RejectCommand(),  # Could potentially be switched with shoot for maximum yeet.
    #         TurnCommand(0.61),  # 35 degrees
    #         SurrogateShooterCommand(),
    #     )

    def interrupted(self):
        pass
