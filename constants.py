from decimal import Decimal, getcontext

from ctre import (
    CANCoderConfiguration,
    AbsoluteSensorRange,
    SensorInitializationStrategy,
)

from wpimath.geometry import Pose2d, Rotation2d, Translation2d

import math


class Constants:
    """
    Dummy class for robot constants.
    """

    pass


"""
Use this class to declare variables that may have to be 
adjusted a lot. This makes it more global and easier to find. 
Please note that this is not the ports.py. That should host 
IDs for the CANbus, sensors, PWM, and the liking. 
"""

drivetrain = Constants()
shooter = Constants()
limelight = Constants()
climber = Constants()

# Drivtrain Cougar Course Decimal Control
drivetrain.decimalPlaces = 36

# Drivetrain Cougar Course graph data
drivetrain.coursesToGraph = {}

# Drive Velocity Control
drivetrain.dPk = 0.0085
drivetrain.dIk = 0
drivetrain.dDk = 0
drivetrain.dFFk = 0.25  # 1?
drivetrain.dIZk = 0

# Drive Position Control
drivetrain.sdPk = 0.5  # 0.1
drivetrain.sdIk = 0
drivetrain.sdDk = 0
drivetrain.sdFFk = 0.18  # 0.15
drivetrain.sdIZk = 0

# Turn Position Control
drivetrain.tPk = 22.5
drivetrain.tIk = 0
drivetrain.tDk = 0.01
drivetrain.tFFk = 0
drivetrain.tIZk = 0

# Turn Secondary Position Control
drivetrain.stPk = 8.5
drivetrain.stIk = 0
drivetrain.stDk = 0
drivetrain.stFFk = 0
drivetrain.stIZk = 0

# PID for holonomic drive controller
drivetrain.hPk = 2
drivetrain.hIk = 0
drivetrain.hDk = 0.03

# Gear ratios on the drivetrain.
drivetrain.driveMotorGearRatio = 6.86
drivetrain.turnMotorGearRatio = 12.8

# Motion magic velocities and accelerations
drivetrain.driveMotionAcceleration = 13500
drivetrain.driveMotionCruiseVelocity = 18500
drivetrain.slowDriveMotionCruiseVelocity = 2000

drivetrain.turnMotionAcceleration = 1000
drivetrain.turnMotionCruiseVelocity = 800

# Trajectory constraints.
drivetrain.maxMetersPerSecond = 1  # Velocity for trajectory
drivetrain.maxMetersPerSecondSquared = 0.1  # Accel for trajectory

# Diameter of the wheel in inches.
drivetrain.wheelDiameter = 4

# Distance between adjacent wheels.
drivetrain.wheelBase = 23.5
drivetrain.trackWidth = 23.5

# Center of the robot to the center of a wheel in inches.
drivetrain.robotRadius = 16.84251

drivetrain.autoSpeedLimit = 2

drivetrain.speedLimit = 2  # meters per second (50 in/s)
drivetrain.maxAcceleration = 1.3  # m/s^2
drivetrain.angularSpeedLimit = math.pi * 2 / 3  # Radians per second
drivetrain.maxAngularAcceleration = math.pi * 1 / 3  # Rad/s^2

# Tolerance of 5 cm and 5 degrees
drivetrain.autoTolerance = Pose2d(Translation2d(0.05, 0.05), Rotation2d.fromDegrees(5))

drivetrain.encoderConfig = CANCoderConfiguration()
drivetrain.encoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360
drivetrain.encoderConfig.initializationStrategy = (
    SensorInitializationStrategy.BootToAbsolutePosition
)
drivetrain.encoderConfig.sensorDirection = False

drivetrain.mostRecentPath = []  # Updated in record auto.

drivetrain.autoPeriodicPeriod = 0.005  # Period to run auto code at a higher rate

# Constants for the shooter below.

shooter.kP = 1.5
shooter.kI = 0
shooter.kD = 0.1
shooter.kF = 0.05
shooter.IZone = 0

# Constants for the limelight below.

# 11 ms, the approximate time taken for a limelight to take a photo
# Add this value to the limelight latency from Network Tables to get total latency,
# which can be used to latency compensate a vision measurement
# See here for more information: https://docs.limelightvision.io/en/latest/networktables_api.html
limelight.captureLatency = 0.011 

limelight.xOffset = 0
limelight.yOffset = 0

limelight.xOffsetStep = 0
limelight.yOffsetStep = 0

limelight.targetHeight = 2.62 # in meters (bottom of tape is 2.58, top is 2.64)
limelight.limelightHeight = 0.635 # meters (height of limelight camera from the ground)
limelight.heightOffset = limelight.targetHeight - limelight.limelightHeight

limelight.limelightAngle = math.radians(10) # 10 degrees (angle in radians)


# Constants for the climber

climber.speed = 1  # 100% Speed
climber.upperLimit = 515000
climber.lowerLimit = 14000  # Give some wiggle room.
