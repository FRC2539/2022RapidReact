"""
This is the place where we store port numbers for all subsystems. It is based on
the RobotMap concept from WPILib. Each subsystem should have its own ports list.
Values other than port numbers should be stored in Config.
"""


class PortsList:
    """Dummy class used to store variables on an object."""

    pass


drivetrain = PortsList()

# The PDP is/should be ID 16.

"""CAN IDs for motors"""
drivetrain.frontLeftDriveID = 0
drivetrain.frontRightDriveID = 2
drivetrain.backLeftDriveID = 1
drivetrain.backRightDriveID = 3

drivetrain.frontLeftTurnID = 4
drivetrain.frontRightTurnID = 6
drivetrain.backLeftTurnID = 5
drivetrain.backRightTurnID = 7

drivetrain.frontLeftCANCoder = 17
drivetrain.frontRightCANCoder = 19
drivetrain.backLeftCANCoder = 18
drivetrain.backRightCANCoder = 20

limelight = PortsList()
limelight.port = 8

shooter = PortsList()
shooter.motorOneID = 14
shooter.motorTwoID = 15

pneumatics = PortsList()
pneumatics.pcmID = 21
pneumatics.forwardChannel = 1
pneumatics.reverseChannel = 2

climber = PortsList()
climber.motorID = 12
