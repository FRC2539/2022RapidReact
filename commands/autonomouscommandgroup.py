from commands2 import InstantCommand
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
from commands.shooter.highgoalspinupcommand import HighGoalSpinupCommand
from commands.shooter.highfenderspinupcommand import HighFenderSpinupCommand
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

from commands.drivetrain.funnyturncommand import FunnyTurnCommand


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

    def twoBallPlus(self):
        speed = 30000
        acc = 16000

        self.addCommands(
            ResetAutoStateCommand(angle=0),
            ParallelRaceGroup(
                IntakeBallsCommandGroup(),
                SequentialCommandGroup(
                    FunnyMoveCommand(1.1, torySlow=speed, toryAcc=acc),
                    WaitCommand(0.55),
                ),
                HighFenderSpinupCommand(),
            ),
            ParallelRaceGroup(
                IntakeBallsCommandGroup(),
                FunnyMoveCommand(-2, angle=13, torySlow=speed, toryAcc=acc),
                HighFenderSpinupCommand(),
            ),
            TurnCommand(math.radians(20)),
            FunnyMoveCommand(-0.2, torySlow=speed, toryAcc=acc),
            ParallelRaceGroup(HighGoalFenderCommand(), WaitCommand(1.6)),
            AutoCollectBallsCommandGroup(pickupTwo=False),
            ParallelRaceGroup(
                LimelightAngleLockCommand(),
                WaitCommand(0.5),
                SetInitialHoodAngleCommand(),
            ),
            FunnyMoveCommand(-1, torySlow=speed, toryAcc=acc),
            ParallelRaceGroup(
                LimelightAngleLockCommand(),
                WaitCommand(0.5),
                SetInitialHoodAngleCommand(),
            ),
            ParallelRaceGroup(HighGoalShootCommand(), WaitCommand(10), IntakeCommand()),
        )

    def twoBallSteal(self):
        speed = 30000
        acc = 16000

        self.addCommands(
            ResetAutoStateCommand(angle=0),
            ParallelRaceGroup(
                HighGoalSpinupCommand(),
                IntakeBallsCommandGroup(),
                SequentialCommandGroup(
                    FunnyMoveCommand(1.2, angle=12, torySlow=speed, toryAcc=acc),
                    FunnyTurnCommand(-13),
                ),
            ),
            ParallelRaceGroup(
                HighGoalShootCommand(),
                WaitCommand(2.7),
                IntakeCommand(),
            ),
            FunnyTurnCommand(20),
            ParallelRaceGroup(
                IntakeBallsCommandGroup(),
                SequentialCommandGroup(
                    FunnyMoveCommand(1.8, angle=-95, torySlow=speed, toryAcc=acc),
                    WaitCommand(0.4),
                ),
            ),
            FunnyTurnCommand(-110, tolerance=10),
            ParallelRaceGroup(
                IntakeBallsCommandGroup(),
                SequentialCommandGroup(
                    FunnyMoveCommand(4.2, angle=0, torySlow=speed, toryAcc=acc),
                    WaitCommand(2),
                ),
            ),
        )

    def twoBall(self):
        speed = 30000
        acc = 16000

        self.addCommands(
            ResetAutoStateCommand(angle=0),
            ParallelRaceGroup(
                HighGoalSpinupCommand(),
                IntakeBallsCommandGroup(),
                SequentialCommandGroup(
                    FunnyMoveCommand(1.5, angle=12, torySlow=speed, toryAcc=acc),
                    FunnyTurnCommand(-13),
                ),
            ),
            ParallelRaceGroup(
                HighGoalShootCommand(),
                WaitCommand(3.5),
                # IntakeCommand(),
            ),
            FunnyTurnCommand(150),
            ResetAutoStateCommand(),
        )

    def oneBall(self):
        speed = 30000
        acc = 16000

        self.addCommands(
            ResetAutoStateCommand(angle=0),
            ParallelRaceGroup(
                SequentialCommandGroup(
                    FunnyMoveCommand(1.5, torySlow=speed, toryAcc=acc),
                ),
            ),
            ParallelRaceGroup(
                HighGoalShootCommand(),
                WaitCommand(5),
            ),
            FunnyTurnCommand(180),
        )

    def oneBallDaisy(self):
        speed = 30000
        acc = 16000

        self.addCommands(
            ResetAutoStateCommand(angle=0),
            ParallelRaceGroup(
                SequentialCommandGroup(
                    FunnyMoveCommand(1.3, torySlow=speed, toryAcc=acc),
                ),
            ),
            ParallelRaceGroup(
                HighGoalShootCommand(),
                WaitCommand(1.8),
            ),
            FunnyMoveCommand(1.5),
        )

    def doNothing(self):
        self.addCommands(
            ResetAutoStateCommand(angle=0),
        )

    def fiveBall(self):
        speed = 30000
        acc = 16000

        # go further at station and shoot from further away on last

        self.addCommands(
            ResetAutoStateCommand(angle=0),
            ParallelRaceGroup(
                HighGoalSpinupCommand(),
                IntakeBallsCommandGroup(),
                SequentialCommandGroup(
                    FunnyMoveCommand(
                        1.15, angle=12, torySlow=speed, toryAcc=acc
                    ),  # 1.23
                    FunnyMoveCommand(0.2, angle=90, torySlow=speed, toryAcc=acc),
                ),
            ),
            ParallelRaceGroup(
                HighGoalShootCommand(),
                WaitCommand(1.15),
                IntakeCommand(),
            ),
            # FunnyMoveCommand(0.1, angle=-90, torySlow=speed, toryAcc=acc),
            ParallelRaceGroup(
                IntakeBallsCommandGroup(),
                FunnyMoveCommand(3, angle=-134, torySlow=speed, toryAcc=acc),
            ),
            ParallelRaceGroup(
                FunnyTurnCommand(63),  # 60
                HighGoalSpinupCommand(),
            ),
            ParallelRaceGroup(
                IntakeBallsCommandGroup(),
                SequentialCommandGroup(
                    FunnyMoveCommand(0.6, torySlow=speed, toryAcc=acc),
                ),
                HighGoalSpinupCommand(),
            ),
            ParallelRaceGroup(
                HighGoalShootCommand(),
                WaitCommand(1.6),
                IntakeCommand(),
                # LimelightAngleLockCommand(),
            ),
            ParallelRaceGroup(
                IntakeBallsCommandGroup(),
                SequentialCommandGroup(
                    FunnyMoveCommand(4.3, angle=-18, torySlow=speed, toryAcc=acc),
                    WaitCommand(0.5),
                ),
            ),
            ParallelRaceGroup(
                IntakeBallsCommandGroup(),
                FunnyMoveCommand(-2.6, angle=-47, torySlow=speed, toryAcc=acc),
                # HighGoalSpinupCommand(),
            ),
            ParallelRaceGroup(
                HighGoalShootCommand(),
                LimelightAngleLockCommand(),
                WaitCommand(5),
                IntakeCommand(),
            ),
        )

    def twoBallSide(self):
        speed = 30000
        acc = 16000

        self.addCommands(
            ResetAutoStateCommand(angle=0),
            ParallelRaceGroup(
                HighGoalSpinupCommand(),
                IntakeBallsCommandGroup(),
                SequentialCommandGroup(
                    FunnyMoveCommand(1.15, angle=12, torySlow=speed, toryAcc=acc),
                    FunnyMoveCommand(0.2, angle=90, torySlow=speed, toryAcc=acc),
                ),
            ),
            ParallelRaceGroup(
                HighGoalShootCommand(),
                WaitCommand(1.9),
                # IntakeCommand(),
            ),
        )

    def fourBall0(self):
        speed = 30000
        acc = 16000

        self.addCommands(
            ResetAutoStateCommand(angle=0),
            ParallelRaceGroup(
                # HighGoalSpinupCommand(),
                IntakeBallsCommandGroup(),
                SequentialCommandGroup(
                    FunnyMoveCommand(1.7, angle=-36, torySlow=6000, toryAcc=acc),
                    FunnyTurnCommand(13),
                ),
            ),
            ParallelRaceGroup(
                HighGoalShootCommand(),
                WaitCommand(2),
                # IntakeCommand(),
            ),
            ParallelRaceGroup(
                IntakeBallsCommandGroup(),
                SequentialCommandGroup(
                    FunnyMoveCommand(4, angle=-34, torySlow=speed, toryAcc=acc),
                    WaitCommand(0.5),
                    FunnyMoveCommand(-3.35, angle=-50, torySlow=speed, toryAcc=acc),
                ),
            ),
            FunnyTurnCommand(6),
            ParallelRaceGroup(
                HighGoalShootCommand(),
                # LimelightAngleLockCommand(),
                WaitCommand(3),
            ),
            FunnyTurnCommand(-150),
            ResetAutoStateCommand(),
        )

    def interrupted(self):
        pass
