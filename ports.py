"""
This is the place where we store port numbers for all subsystems. It is based on
the RobotMap concept from WPILib. Each subsystem should have its own ports list.
Values other than port numbers should be stored in Config.
"""


class PortsList:
    """Dummy class used to store variables on an object."""

    pass


drivetrain = PortsList()

"""CAN IDs for motors"""
drivetrain.frontLeftDriveID = 0
drivetrain.frontRightDriveID = 2
drivetrain.backLeftDriveID = 1
drivetrain.backRightDriveID = 3

drivetrain.frontLeftTurnID = 4
drivetrain.frontRightTurnID = 6
drivetrain.backLeftTurnID = 5
drivetrain.backRightTurnID = 7

drivetrain.frontLeftCANCoder = 24
drivetrain.frontRightCANCoder = 26
drivetrain.backLeftCANCoder = 25
drivetrain.backRightCANCoder = 27

limelight = PortsList()
limelight.port = 8

shooter = PortsList()
shooter.motorOneID = 8
shooter.motorTwoID = 9

pneumatics = PortsList()
pneumatics.pcmID = 17

climber = PortsList()
climber.motorID = 18
climber.forwardChannel = 6
climber.reverseChannel = 7

intake = PortsList()
intake.motorID = 10
intake.forwardChannel = 0
intake.reverseChannel = 1

hood = PortsList()
hood.motorID = 13

ballsystem = PortsList()
ballsystem.conveyorMotorID = 11
ballsystem.chamberMotorID = 12
ballsystem.conveyorSensor = 0

ml = PortsList()

lights = PortsList()
lights.lightControllerID = 9  # PWM

compressor = PortsList()
