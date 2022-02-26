from .logitechdualshock import LogitechDualShock
from .thrustmasterjoystick import ThrustmasterJoystick
from . import logicalaxes

from commands2 import InstantCommand

from commands2 import ParallelCommandGroup


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
from commands.ballsystem.forwardballsystemcommand import ForwardBallSystemCommand

# from commands.drivetrain.autocollectballscommand import AutoCollectBallsCommand
from commands.drivetrain.autocollectballscommandgroup import (
    AutoCollectBallsCommandGroup,
)

from commands.limelight.limelightanglelockcommand import LimelightAngleLockCommand
from commands.drivetrain.enablelimelightlockcommand import EnableLimelightLockCommand


from commands.lights.lightsbluecommand import LightsBlueCommand
from commands.lights.lightsorangecommand import LightsOrangeCommand
from commands.lights.lightsseizurecommand import LightsSeizureCommand

from commands.hood.raisehoodcommand import RaiseHoodCommand
from commands.hood.lowerhoodcommand import LowerHoodCommand
from commands.hood.manualsethoodpositioncommand import ManualSetHoodPositionCommand

from commands.shooter.setshooterrpmscommand import SetShooterRPMsCommand
from commands.shooter.lowgoalshootcommand import LowGoalShootCommand

# from commands.shooter.highgoalshootcommand import HighGoalShootCommand
from commands.shooter.highgoalfendercommand import HighGoalFenderCommand


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

    # Replace with the automated commands
    driveControllerOne.RightThumb.whileHeld(RaiseClimberCommand())
    driveControllerOne.LeftThumb.whileHeld(LowerClimberCommand())
    driveControllerOne.BottomThumb.whenPressed(ToggleClimberSolenoidCommand())

    driveControllerOne.LeftBottomLeft.whileHeld(RaiseClimberCommand())
    driveControllerOne.LeftBottomMiddle.whileHeld(LowerClimberCommand())
    driveControllerOne.LeftBottomRight.whileHeld(ToggleClimberSolenoidCommand())

    # driveControllerOne.Trigger.whileHeld(SetShooterRPMsCommand(1350, 1000))
    # driveControllerOne.Trigger.whileHeld(HighGoalShootCommand())
    driveControllerOne.Trigger.whileHeld(HighGoalFenderCommand())

    # Reverse lower shot
    # driveControllerOne.Trigger.whileHeld()

    driveControllerTwo.LeftThumb.whileHeld(IntakeBallsCommandGroup())
    driveControllerTwo.RightThumb.whileHeld(AutoCollectBallsCommandGroup())
    driveControllerTwo.BottomThumb.whileHeld(EnableLimelightLockCommand())

    # driveControllerTwo.Trigger.whileHeld(
    #     ParallelCommandGroup(
    #         SetShooterRPMsCommand(1350, 1000),
    #         ForwardBallSystemCommand(),
    #     )
    # )

    driveControllerTwo.Trigger.whileHeld(LowGoalShootCommand())

    # driveControllerTwo.LeftTopLeft.whileHeld()
    # driveControllerTwo.LeftBottomLeft.whileHeld()

    # driveControllerOne.LeftTopRight.whileHeld()
    driveControllerOne.LeftTopRight.whenPressed(LightsSeizureCommand())
    # driveControllerOne.LeftBottomRight.whileHeld(ClimbBarCommand())
    # driveControllerOne.LeftTopLeft.whileHeld()
    # driveControllerOne.LeftTopLeft.whenPressed(LightsOrangeCommand())
    driveControllerOne.LeftTopLeft.whenPressed(ResetAutoStateCommand())

    # The controller for non-driving subsystems of the robot
    componentController = LogitechDualShock(2)

    # logicalaxes.TURRETmOVE = componentController.LeftX

    componentController.Back.whenPressed(ResetCommand())

    componentController.LeftTrigger.whileHeld(LowerHoodCommand())
    componentController.LeftBumper.whileHeld(RaiseHoodCommand())
    # componentController.RightTrigger.whileHeld(HighGoalShootCommand())
    # componentController.RightBumper.whileHeld()

    # componentController.A.whileHeld()
    componentController.X.whileHeld(RejectCommand())
    componentController.B.whileHeld(RejectCommand())
    # componentController.Y.whileHeld()

    # componentController.DPadUp.whenPressed()
    # componentController.DPadDown.whenPressed()
    # componentController.DPadRight.whenPressed()
    # componentController.DPadLeft.whenPressed()

    # componentController.Start.toggleWhenPressed()
