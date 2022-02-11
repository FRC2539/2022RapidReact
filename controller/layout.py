from ast import For
from .logitechdualshock import LogitechDualShock
from .thrustmasterjoystick import ThrustmasterJoystick
from . import logicalaxes

from commands2 import InstantCommand


from commands.drivetrain.drivecommand import DriveCommand

from commands.drivetrain.togglefieldorientationcommand import (
    ToggleFieldOrientationCommand,
)
from commands.drivetrain.zerocancoderscommand import ZeroCANCodersCommand
from commands.drivetrain.setspeedcommand import SetSpeedCommand

from commands.drivetrain.zerogyrocommand import ZeroGyroCommand

from commands.drivetrain.resetautostatecommand import ResetAutoStateCommand

from commands.resetcommand import ResetCommand

from commands.intake.intakecommand import IntakeCommand
from commands.intake.rejectcommand import RejectCommand

from commands.climber.raiseclimbercommand import RaiseClimberCommand
from commands.climber.lowerclimbercommand import LowerClimberCommand
from commands.climber.toggleclimbersolenoidcommand import ToggleClimberSolenoidCommand
from commands.climber.climbbarcommand import ClimbBarCommand

from commands.intakeballscommandgroup import IntakeBallsCommandGroup
from commands.ballsystem.forwardconveyorcommand import ForwardConveyorCommand

from commands.drivetrain.autocollectballscommand import AutoCollectBallsCommand

import constants
import robot


def init():
    """
    Declare all controllers, assign axes to logical axes, and trigger
    commands on various button events. Available event types are:
        - whenPressed
        - whileHeld: cancelled when the button is released
        - whenReleased
        - toggleWhenPressed: start on first press, cancel on next
        - cancelWhenPressed: good for commands started with a different button
    """

    # The controller for driving the robot
    driveControllerOne = ThrustmasterJoystick(0)  # The left hand controller
    driveControllerTwo = ThrustmasterJoystick(1)  # The right hand controller

    # Define the x, y, and rotate and rotate axes
    logicalaxes.forward = driveControllerOne.Y
    logicalaxes.strafe = driveControllerOne.X
    logicalaxes.rotate = driveControllerTwo.X

    driveControllerOne.RightThumb.whileHeld(ForwardConveyorCommand())
    # driveControllerOne.RightThumb.whileHeld(RaiseClimberCommand())
    # driveControllerOne.LeftThumb.whileHeld(LowerClimberCommand())

    # driveControllerOne.BottomThumb.whenPressed(ToggleClimberSolenoidCommand())

    driveControllerOne.Trigger.whileHeld(IntakeBallsCommandGroup())

    driveControllerTwo.LeftThumb.whileHeld(IntakeCommand())
    driveControllerTwo.RightThumb.whileHeld(RejectCommand())

    driveControllerTwo.BottomThumb.whileHeld(AutoCollectBallsCommand())

    # driveControllerTwo.Trigger.whileHeld()

    # driveControllerTwo.LeftTopLeft.whileHeld()
    # driveControllerTwo.LeftBottomLeft.whileHeld()

    # driveControllerOne.LeftTopRight.whileHeld()
    # driveControllerOne.LeftBottomRight.whileHeld(ClimbBarCommand())
    # driveControllerOne.LeftTopLeft.whileHeld()
    driveControllerOne.LeftBottomLeft.whenPressed(ResetAutoStateCommand())

    # driveControllerOne.RightTopLeft.whileHeld()
    # driveControllerOne.RightBottomLeft.whileHeld()
    # driveControllerOne.RightBottomRight.whileHeld()

    # driveControllerOne.RightBottomLeft.whileHeld()
    # driveControllerOne.RightBottomMiddle.whenPressed()
    # driveControllerOne.RightBottomRight.whileHeld()
    # driveControllerOne.RightTopRight.whenPressed(AutomatedColorControlCommand())

    # driveControllerTwo.LeftBottomMiddle.whileHeld()
    # driveControllerTwo.LeftTopMiddle.whileHeld()
    # driveControllerTwo.LeftTopMiddle.whileHeld()

    # The controller for non-driving subsystems of the robot
    componentController = LogitechDualShock(2)

    # logicalaxes.TURRETmOVE = componentController.LeftX

    componentController.Back.whenPressed(ResetCommand())

    # componentController.LeftTrigger.whileHeld()
    # componentController.LeftBumper.whileHeld()
    # componentController.RightTrigger.whileHeld()
    # componentController.RightBumper.whileHeld()

    # componentController.A.whenPressed()
    # componentController.X.whileHeld()
    # componentController.B.whileHeld()
    # componentController.Y.whileHeld()

    # componentController.DPadUp.whenPressed()
    # componentController.DPadDown.whenPressed()
    # componentController.DPadRight.whenPressed()
    # componentController.DPadLeft.whenPressed()

    # componentController.Start.toggleWhenPressed()
