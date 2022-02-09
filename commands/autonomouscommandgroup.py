from wpilib import DriverStation

from commands2 import SequentialCommandGroup

from networktables import NetworkTables

import math
import robot, constants

from commands import autoconfig

from commands.drivetrain.movecommand import MoveCommand
from commands.drivetrain.resetautostatecommand import ResetAutoStateCommand
from commands.drivetrain.turncommand import TurnCommand
from commands.drivetrain.turninplacecommand import TurnInPlaceCommand

from commands.drivetrain.trajectoryfollowercommand import TrajectoryFollowerCommand


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

    def moveAndTurn0(self):
        self.addCommands(
            ResetAutoStateCommand(x=0, y=0, angle=0),
            MoveCommand(1, linearVelocity=0.5, matchHeading=False),
            # TurnCommand(90),
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
            MoveCommand(1, linearVelocity=0.5, matchHeading=False),
            TurnInPlaceCommand(6.28, accelerationRate=4, turnSpeed=1),
        )

    def interrupted(self):
        pass
