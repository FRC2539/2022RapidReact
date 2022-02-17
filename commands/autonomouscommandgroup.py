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

    def fiveBall(self):
        """Immediately shoots a red ball, collects 2 balls and shoots them, then collects a red ball + one from the human player station."""
        self.addCommands(
            ResetAutoStateCommand(x=0, y=0, angle=-1.57), # -90 degrees
            SurrogateShooterCommand(),
            InstantCommand(lambda: robot.intake.IntakeCommand(), [robot.intake]),
            CustomMoveCommand(x=0, y=),
            TurnCommand(-1.76), # -101 degrees
            CustomMoveCommand(),
        )
    
    def fourBall(self):
        """Collect the leftmost red ball, shoot 2, then collect 2 more balls and shoot those."""
        self.addCommands(
            #Collect the leftmost ball.
            ResetAutoStateCommand(x=0, y=0, angle=-1.57), # -90 degrees.
            InstantCommand(lambda: robot.intake.IntakeCommand(), [robot.intake]),
            CustomMoveCommand(x=0, y=1.89),
            
            #Collect the remaining two balls.
            CustomMoveCommand(x=0, y=-1.89),
            TurnCommand(turnAngle=-1.46),
            InstantCommand(lambda: robot.intake.IntakeCommand(), [robot.intake]),
            CustomMoveCommand(x=6.67, y=0.71),
            
            #Return to shoot.
            CustomMoveCommand(x=-2.69, y=0.29),
            TurnCommand(0.35), #20 degrees
            SurrogateShooterCommand(),
        )

    def twoBallLeftND(self):
        """Collects the leftmost red ball and shoots both. ND standss for Non-Disruptive (no blue ball interference)."""
        self.addCommands(
            ResetAutoStateCommand(x=0, y=0, angle=-1.57), #-90 degrees
            InstantCommand(lambda: robot.intake.IntakeCommand(), [robot.intake]),
            CustomMoveCommand(x=0, y=1.89),
            SurrogateShooterCommand(),
        )

    def twoBallMidND(self):
        """Collects the middle red ball and shoots both. ND stands for Non-Disruptive (no blue ball interference)."""
        self.addCommands(
            ResetAutoStateCommand(x=0, y=0, angle=-2.15), # -123 degrees
            InstantCommand(lambda: robot.intake.IntakeCommand(), [robot.intake]),
            CustomMoveCommand(x=1.23, y=0.6),
            TurnCommand(0.54), #31 degrees
            SurrogateShooterCommand(),
        )

    def twoBallRightND(self):
        """Collects the rightmost red ball and shoots both. ND stands for Non-Disruptive (no blue ball interference)"""
        self.addCommands(
            ResetAutoStateCommand(x=0, y=0, angle=2.30), # 132 degrees
            InstantCommand(lambda: robot.intake.IntakeCommand(), [robot.intake]),
            CustomMoveCommand(x=0.75, y=-0.83),
            TurnCommand(0.17), #10 degrees
            SurrogateShooterCommand(),
        )
    
    def twoBallLeftYD(self):
        """Collects the rightmost blue ball and yeets it away from the centerline before scoring 1 red ball. YD stands for Yes-Disruptive (blue ball interference)."""
        self.addCommands(
            #Collect and yeet the blue ball.
            ResetAutoStateCommand(x=0, y=0, angle=-1.27), # -73 degrees
            InstantCommand(lambda: robot.intake.IntakeCommand(), [robot.intake]),
            CustomMoveCommand(x=-0.35, y=1.16),
            TurnCommand(-2.04), # -117 degrees
            InstantCommand(lambda: robot.intake.RejectCommand(), [robot.intake]), #Could potentially be switched with shoot for maximum yeet.
           
            #Shoot the red ball.
            TurnCommand(1.95), # 112 degrees
            SurrogateShooterCommand(),
        )
    
    def twoBallRightYD(self):
        """Collects the leftmost blue ball and yeets it away from the centerline before scoring 1 red ball. YD stands for Yes-Disruptive (blue ball interference)."""
        self.addCommands(
            #Collect and yeet the blue ball.
            ResetAutoStateCommand(x=0, y=0, angle=-2.06), # -118 degrees
            InstantCommand(lambda: robot.intake.IntakeCommand(), [robot.intake]),
            CustomMoveCommand(x=1.16, y=0.35),
            TurnCommand(-0.79), # -45 degrees
            InstantCommand(lambda: robot.intake.RejectCommand(), [robot.intake]), #Could potentially be switched with shoot for maximum yeet.
            
            TurnCommand(0.61), #35 degrees
            SurrogateShooterCommand(),
        )
        
    def interrupted(self):
        pass
