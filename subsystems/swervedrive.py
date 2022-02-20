from decimal import Decimal, getcontext

import math

from wpimath.kinematics import (
    SwerveDrive4Kinematics,
    SwerveModuleState,
    ChassisSpeeds,
)

from wpimath.estimator import SwerveDrive4PoseEstimator

from wpimath.trajectory import (
    TrajectoryGenerator,
    TrajectoryConfig,
    TrapezoidProfileRadians,
)

from wpilib import Timer

from wpimath.controller import HolonomicDriveController
from wpimath.controller import PIDController, ProfiledPIDControllerRadians

from wpimath.geometry import Translation2d, Rotation2d, Pose2d

from controller import logicalaxes

from .cougarsystem import *
from .basedrive import BaseDrive
from .swervemodule import SwerveModule

from custom.rref import Matrix

import ports
import constants

from networktables import NetworkTables

logicalaxes.registerAxis("forward")
logicalaxes.registerAxis("strafe")


class SwerveDrive(BaseDrive):
    """
    "Rollers? Where we're going, we don't need 'rollers'." - Ben Bistline, 2021

    A drivetrain class for swerve drive.
    """

    def __init__(self):
        """
        The constructor for the class. When returning lists, it should follow like:
        [front left, front right, back left, back right]
        """

        super().__init__("Swerve Drive")

        self.isFieldOriented = True

        self.wheelBase = (
            constants.drivetrain.wheelBase
        )  # These are distances across the robot; horizontal, vertical, diagonal.
        self.trackWidth = constants.drivetrain.trackWidth
        self.r = math.sqrt(self.wheelBase ** 2 + self.trackWidth ** 2)

        self.speedLimit = (
            constants.drivetrain.speedLimit
        )  # Override the basedrive without editing the file.

        self.angularSpeedLimit = constants.drivetrain.angularSpeedLimit

        # Creates a list of swerve modules.
        self.modules = [
            SwerveModule(  # Front left module.
                "front left",
                ports.drivetrain.frontLeftDriveID,
                ports.drivetrain.frontLeftTurnID,
                ports.drivetrain.frontLeftCANCoder,
                self.speedLimit,
                299.434,
                # 321.416,  # -90,  # -255.498047,
                180,  # Offset basis - used for zeroing CANCoder
            ),
            SwerveModule(  # Front right module.
                "front right",
                ports.drivetrain.frontRightDriveID,
                ports.drivetrain.frontRightTurnID,
                ports.drivetrain.frontRightCANCoder,
                self.speedLimit,
                191.78,
                # 182.373,  # 15,  # -272.548840625,
                360,  # Offset basis - used for zeroing CANCoder
                invertedDrive=True,  # Invert for some reason?
            ),
            SwerveModule(  # Back left module.
                "back left",
                ports.drivetrain.backLeftDriveID,
                ports.drivetrain.backLeftTurnID,
                ports.drivetrain.backLeftCANCoder,
                self.speedLimit,
                20.033,
                # 349.277,  # 10,  # -40.8692515625,
                -180,  # Offset basis - used for zeroing CANCoder
            ),
            SwerveModule(  # Back right module.
                "back right",
                ports.drivetrain.backRightDriveID,
                ports.drivetrain.backRightTurnID,
                ports.drivetrain.backRightCANCoder,
                self.speedLimit,
                84.915,
                # 4.131,  # 90,  # -128.759766125,
                -360,  # Offset basis - used for zeroing CANCoder
                invertedDrive=True,  # Invert for some reason. Ezra's going nuts lol.
            ),
        ]

        self.swerveKinematics = (
            SwerveDrive4Kinematics(  # X and Y components of center offsets.
                Translation2d(0.427799754, 0.427799754),  # Front left module
                Translation2d(0.427799754, -0.427799754),  # Front right module
                Translation2d(-0.427799754, 0.427799754),  # Back left module
                Translation2d(-0.427799754, -0.427799754),  # Back right module
            )
        )

        for module in self.modules:  # Add the motors to the robot's orchestra.
            self.addOrchestraInstrument(module.getDriveMotor())
            self.addOrchestraInstrument(module.getTurnMotor())

        self.swervePoseEstimator = SwerveDrive4PoseEstimator(
            self.getRotationForPoseEstimator(),
            Pose2d(),  # Default the starting location to (0, 0) theta = 0
            self.swerveKinematics,
            [0, 0, 0],
            [0],
            [0, 0, 0],
            nominalDt=0.02,
        )

        # State variable for the drive command
        self.limelightLock = False  # Rotation lock towards the limelight target

        self.resetGyro()
        self.resetPoseEstimate()

        getcontext().prec = constants.drivetrain.decimalPlaces

        self.put("wheelAngles", self.getModuleAngles())
        self.put("wheelSpeeds", self.getSpeeds())
        self.put("robotVector", [0, 0])
        self.put("correctedOffsets", [0])

        # Stores whether or not the robot should send over the corrected wheel angle offsets
        self.sendOffsets = False

        # Sync that value with network tables
        self.put("sendOffsets", self.sendOffsets)

        self.hPk = constants.drivetrain.hPk
        self.hIk = constants.drivetrain.hIk
        self.hDk = constants.drivetrain.hDk

        xController = PIDController(self.hPk, self.hIk, self.hDk)
        yController = PIDController(self.hPk, self.hIk, self.hDk)

        # Create a theta controller used for autonomous
        thetaController = ProfiledPIDControllerRadians(
            constants.drivetrain.htPk,
            constants.drivetrain.htIk,
            constants.drivetrain.htDk,
            TrapezoidProfileRadians.Constraints(
                constants.drivetrain.angularSpeedLimit,
                constants.drivetrain.maxAngularAcceleration,
            ),
        )
        thetaController.enableContinuousInput(-math.pi, math.pi)

        # Create a theta controller used for driving
        self.driveThetaController = ProfiledPIDControllerRadians(
            3.5,
            0,
            0.035,
            TrapezoidProfileRadians.Constraints(
                math.pi * 1.5,
                math.pi * 1.25,
            ),
        )
        self.driveThetaController.enableContinuousInput(-math.pi, math.pi)

        # Create a drive controller for autonomous
        self.driveController = HolonomicDriveController(
            xController,
            yController,
            thetaController,
        )

        self.trajectoryConfig = TrajectoryConfig(
            constants.drivetrain.autoSpeedLimit, constants.drivetrain.maxAcceleration
        )

        self.trajectoryConfig.setKinematics(self.swerveKinematics)

        # Create a sample trajectory
        self.trajectory = TrajectoryGenerator.generateTrajectory(
            Pose2d(0, 0, Rotation2d(0)),
            [
                Translation2d(-1, 0),
                Translation2d(-1, 1),
            ],
            Pose2d(-2, 1, Rotation2d(0)),
            self.trajectoryConfig,
        )

    def periodic(self):
        """
        Loops whenever there is robot code. I recommend
        feeding networktable values here.
        """

        self.feed()

        # Update's the robot's pose estimate.
        self.updatePoseEstimate()

        # Update networktables.
        self.put("wheelAngles", self.getModuleAngles())
        self.put("wheelSpeeds", self.getSpeeds())

        x, y = self.generateRobotVector()

        r = abs(math.sqrt(x ** 2 + y ** 2)) / 12  # Provides speed in fps.
        theta = (math.atan2(y, x) * 180 / math.pi) - 180  # Provides angle in degrees.

        self.put("robotVector", [r, theta])

        controllerR = math.sqrt(
            logicalaxes.forward.get() ** 2 + logicalaxes.strafe.get() ** 2
        )

        self.put("joystickPercent", controllerR)
        self.put("wheelPercents", self.getPercents())

        self.updateSendOffsetsState()

        # Send the corrected wheel offsets
        # if that has been indicated in the dashboard
        if self.sendOffsets:
            self.put("correctedOffsets", self.getCorrectedModuleOffsets())

    def enableLimelightLock(self):
        self.limelightLock = True

    def disableLimelightLock(self):
        self.limelightLock = False

    def isLimelightLockEnabled(self):
        return self.limelightLock

    def debugPrints(self):
        print("-----------------")
        print(f"Angle: {self.getAngle()}")
        print("Swerve Pose:")
        print(self.getSwervePose())
        print("-----------------")

    def updateSendOffsetsState(self):
        self.sendOffsets = self.get("sendOffsets")

    def generateRobotVector(self):
        """
        Creates vectors for each module depending
        on that module's speed and direction. Used to
        calculate our odometry.
        """
        Angles = self.getModuleAngles()
        Speeds = self.getSpeeds()
        VectorX = 0
        VectorY = 0
        for angle, speed in zip(Angles, Speeds):
            VectorX += math.cos(math.radians(angle - 180)) * speed
            VectorY += math.sin(math.radians(angle - 180)) * speed
        VectorX = VectorX / 4
        VectorY = VectorY / 4
        return VectorX, VectorY

    def waitForRoll(self):
        """
        Forces the robot to wait until
        it's not tipping.
        """
        while abs(self.navX.getRoll()) > 5:
            pass

    def updatePoseEstimate(self):
        """
        Updates the WPILib pose estimate object
        using the gyro and the module states.
        """

        states = self.getModuleStates()

        self.swervePoseEstimator.update(
            self.getRotationForPoseEstimator(),
            states[0],  # 0
            states[1],  # 1
            states[2],  # 2
            states[3],  # 3
        )

    def getRotationForPoseEstimator(self):
        """
        Flips the measurement from the navX 180 degrees.
        This is necessary for pose estimation.
        """
        # return Rotation2d(self.navX.getRotation2d().radians() * -1)
        return self.navX.getRotation2d()

    def addVisionPoseEstimate(self, pose, latency):
        """
        Updates the pose estimator using a pose estimate
        coming from a vision system.

        Parameters:
            pose - a Pose2d object
            latency - (sec) how long ago the measurement was taken (i.e. limelight latency)
        """
        # Calculate the time the image data was captured
        timestamp = Timer.getFPGATimestamp() - latency

        self.swervePoseEstimator.addVisionMeasurement(pose, timestamp)

    def resetPoseEstimate(self, pose=Pose2d()):
        """
        Resets the pose estimate to a given position, typically the one used when starting a trajectory.
        """
        self.resetEncoders()
        self.swervePoseEstimator.resetPosition(pose, self.getRotationForPoseEstimator())

    def getSwervePose(self):
        """
        Get the current pose estimate
        """
        return self.swervePoseEstimator.getEstimatedPosition()

    def getPoseRotation(self):
        """
        Get the angle of the robot from the pose estimate (kalman filter),
        rather than from the gyro alone.
        """
        return self.getSwervePose().rotation()

    def getPoseTranslation(self):
        """
        Get the translation of the robot from the pose estimate (kalman filter),
        rather than from the encoders alone.
        """
        return self.getSwervePose().translation()

    def getChassisSpeeds(self):
        """
        Returns the robots velocity and heading, using
        module states, in the form of a ChassisSpeeds object.
        """

        return self.swerveKinematics.toChassisSpeeds(self.getModuleStates())

    def getChassisSpeedsData(self):
        """
        Basically the same thing as getChassisSpeeds, but this one
        extracts the data and returns the useful stuff in a list, which
        looks like this: [vx_fps, vy_fps, omega_dps].
        """

        speeds = self.swerveKinematics.toChassisSpeeds(self.getModuleStates())

        return [speeds.vy_fps, -speeds.vx_fps, speeds.omega_dps]

    def _configureMotors(self):
        """
        Configures the motors. Shouldn't need this.
        """

        self.activeMotors = self.motors[
            0:2
        ]  # Don't actually need these, this just keeps basedrive happy.

    def _calculateSpeeds(self, x, y, rotate):
        pass

    def move(self, x, y, rotate):
        """
        Turns coordinate arguments into motor outputs.
        Short-circuits the rather expensive movement calculations if the
        coordinates have not changed.
        """

        """Prevent drift caused by small input values"""
        x = math.copysign(max(abs(x) - self.deadband, 0), x)
        y = math.copysign(max(abs(y) - self.deadband, 0), y)
        rotate = math.copysign(max(abs(rotate) - (self.deadband + 0.05), 0), rotate)

        if [x, y, rotate] == [0, 0, 0]:
            self.stop()
            return

        targetChassisSpeeds = self.convertControllerToChassisSpeeds(x, y, rotate)

        # print("speeds")
        # print(targetChassisSpeeds)
        # print("angle")
        # print(self.navX.getRotation2d())
        # print()

        self.setChassisSpeeds(targetChassisSpeeds)

    def convertControllerToChassisSpeeds(self, x, y, rotate):
        # Convert the percent outputs from the joysticks
        # to meters per second and radians per second
        vx, vy, omega = (
            x * self.speedLimit,
            y * self.speedLimit,
            rotate * self.angularSpeedLimit,
        )

        # Convert and return the target velocities to a chassis speeds object
        return self.getRobotRelativeSpeedsFromFieldSpeeds(vx, vy, omega)

    def getRobotRelativeSpeedsFromFieldSpeeds(self, vx, vy, omega):
        """
        Converts and returns the target velocities to a chassis speeds object
        """
        return ChassisSpeeds.fromFieldRelativeSpeeds(
            vx, vy, omega, self.navX.getRotation2d()
        )

    def getRobotRelativeSpeedsFromFieldSpeedsReversed(self, vx, vy, omega):
        """
        Converts and returns the target velocities to a chassis speeds object
        """
        return ChassisSpeeds.fromFieldRelativeSpeeds(
            vx, vy, omega, Rotation2d(self.navX.getRotation2d().radians() * -1)
        )

    def setChassisSpeeds(self, chassisSpeeds):
        """
        Converts a chassis speeds object to module states,
        and then set the swerve module to those states.
        """

        moduleStates = self.convertChassisSpeedsToModuleStates(chassisSpeeds)

        optimizedModuleStates = self.optimizeModuleStates(moduleStates)

        # Set the swerve modules to the module states
        self.setModuleStates(optimizedModuleStates)
        # self.setModuleStates(moduleStates)

    def setChassisSpeedsRaw(self, chassisSpeeds):
        """
        Converts a chassis speeds object to module states,
        and then set the swerve module to those states.
        """

        moduleStates = self.convertChassisSpeedsToModuleStates(chassisSpeeds)

        # Set the swerve modules to the module states
        self.setModuleStates(moduleStates)

    def optimizeModuleStates(self, moduleStates):
        optimizedStates = []

        # Optimize the module states to reduce how much we
        # need to change the heading of the wheels
        for module, moduleState in zip(self.modules, moduleStates):
            optimizedState = SwerveModuleState.optimize(
                moduleState, Rotation2d.fromDegrees(module.getWheelAngle())
            )

            optimizedStates.append(optimizedState)

        return optimizedStates

    def convertChassisSpeedsToModuleStates(self, chassisSpeeds):
        # Use inverse kinematics to convert the general chassis speeds
        # object to module states for each swerve module
        return self.swerveKinematics.toSwerveModuleStates(chassisSpeeds)

    def getModuleStates(self):
        """
        Returns the module state objects that represent each swerve module
        """

        return [module.getModuleState() for module in self.modules]

    def setModuleStates(self, moduleStates):
        """
        Set the states of the modules based on general swerve module state objects
        """
        for module, state in zip(self.modules, moduleStates):
            module.setModuleState(state)

    def tankMove(self, y, rotate):
        """
        Drive the robot in a tank style. This does not work however because
        of physical constraints. The code works however.
        """
        if [y, rotate] == self.lastInputs:
            return

        self.lastInputs = [y, rotate]

        """Prevent drift caused by small input values"""
        if self.useEncoders:
            y = math.copysign(max(abs(y) - self.deadband, 0), y)
            rotate = math.copysign(max(abs(rotate) - self.deadband, 0), rotate)

        speeds = self.tankCalculateSpeeds(y, rotate)

        for module, speed in zip(self.modules, speeds):
            module.setWheelAngle(0)
            module.setWheelSpeed(speed)

    def tankCalculateSpeeds(self, y, rotate):
        """
        Calculate the wheel speeds for the drivetrain when acting as a
        tank drivetrain. Again, does not work because of physical constraints.
        """
        return [y + rotate, -y + rotate, y + rotate, -y + rotate]  # FL, FR, BL, BR

    def stop(self):
        """
        Stops both motors of each of the modules.
        """
        for module in self.modules:
            module.stopModule()

    def longStop(self):
        """
        Returns true when all wheel speeds
        are zero.
        """
        self.stop()
        while self.getSpeeds().count(0) < 3:
            pass

    def resetEncoders(self, anArgumentAsWell=0):
        """
        Resets all drive encoders to 0 by default.
        """
        for module in self.modules:
            module.resetDriveEncoders(anArgumentAsWell)

    def setProfile(self, profile):
        """
        Sets the profile for both drive and turn motors.
        """
        for module in self.modules:
            module.setModuleProfile(profile)

    def setModuleProfiles(self, id_=0, drive=True, turn=True):
        """
        Sets the PID profiles for each of the modules.
        This one accepts an optional turn and drive.
        """
        for module in self.modules:
            module.setModuleProfile(id_, drive, turn)

    def updateCANCoders(self, positions: list):
        """
        Sets the position of the CANCoders. Be careful using
        this method!
        """
        for module, position in zip(self.modules, positions):
            module.updateCANCoder(position)

    def setSpeedLimit(self, speed):
        """
        Sets the speed limit of the drive motor in
        inches per second.
        """
        self.speedLimit = speed

        for module in self.modules:
            module.speedLimit = speed

    def setFieldOriented(self, fieldCentric=True):
        """
        Changes the orientation of the robot. It should almost always be
        field centric on a swerve robot.
        """
        self.isFieldOriented = fieldCentric

    def getSpeeds(self, inIPS=True):  # Defaults to giving in inches per second.
        """
        Returns the speeds of the wheel.
        """
        return [module.getWheelSpeed() for module in self.modules]

    def setSpeeds(self, speeds: list):  # Set a speed in inches per second.
        """
        Sets the speeds of the wheels in inches per second.
        It takes a list. Please use setUniformModuleSpeed if
        you want to set the same speed amongst all the modules.
        """
        for module, speed in zip(self.modules, speeds):
            module.setWheelSpeed(speed)

    def setUniformModuleSpeed(self, speed: float):  # Set a speed in inches per second.
        """
        Sets a uniform speed to eall the drive motors in inches per
        second. This takes a float because all modules use
        the same speed. Use the setSpeeds method if you want to pass
        a list of different speeds.
        """
        for module in self.modules:
            module.setWheelSpeed(speed)

    def getPercents(self):
        """
        Returns the percent outputs of each drive motor.
        """
        return [module.getWheelPercent() for module in self.modules]

    def setPercents(self, speeds: list):
        """
        Sets the percent speed of each module's drive motor.
        """
        for module, speed in zip(self.modules, speeds):
            module.setWheelPercent(speed)

    def setUniformModulePercent(self, speed: float):
        """
        Sets a uniform percent to the drive motor
        of each module.
        """
        for module in self.modules:
            module.setWheelPercent(speed)

    def getModuleAngles(self):
        """
        Returns the CANCoder's absolute reading.
        Note, this does take into account the magnet
        offset which we set at the beginning.
        I think, 180 is forward, 0 is backwards. It
        returns between 0 and 360.
        """

        # Add module in front, not to be confused with gyro! Returns degrees.
        return [module.getWheelAngle() % 360 for module in self.modules]

    def printAbsoluteModuleAngles(self):
        """
        Outputs the absolute wheel angles
        for each swerve module to the console.
        """
        angleStrings = [
            f"{module.moduleName}: {module.getAbsoluteWheelAngle()} "
            for module in self.modules
        ]

        print("".join(angleStrings))

    def getCorrectedModuleOffsets(self):
        """
        Determines how far off the offset for each module is.
        """

        # Calculate the new offset for each swerve module
        # Algorithm: Current offset + (Base angle - absolut
        correctedAngles = [
            module.offset + (module.offsetBasis - module.getAbsoluteWheelAngle())
            for module in self.modules
        ]

        return correctedAngles

    def printCorrectedModuleOffsets(self):
        """
        Use this when zeroing the wheels.

        Calculates and outputs what the new offset values should be.

        These values SHOULD BE SIMILAR to the previous values.
        If they aren't, try zeroing the wheels differently.
        """

        correctedAngles = self.getCorrectedModuleOffsets()

        angleStrings = [
            f"{module.moduleName}: {angle} "
            for module, angle in zip(self.modules, correctedAngles)
        ]

        print("".join(angleStrings))

    def setModuleAngles(self, angles: list):  # Set a list of different angles.
        """
        Set the angle of the wheel using the turn motor.
        This method takes a list of angles, 0-360 degrees.
        """

        for module, angle in zip(self.modules, angles):
            module.setWheelAngle(angle)

    def setUniformModuleAngle(self, angle: int):
        """
        Set the angle of the wheel using the turn motor.
        This method takes a universal angle to set to all
        modules. The angle should be 0-360 degrees.
        """
        for module in self.modules:
            module.setWheelAngle(angle)

    def getPositions(self, inInches=True):  # Defaults to giving in inches.
        """
        Returns the module position in inches (by default).
        """
        return [module.getModulePosition(inInches) for module in self.modules]

    def setPositions(self, positions: list):
        """
        Sets the position of the modules. It will go forward this many inches.
        I recommend using the setUniformModulePosition however.
        Remember, provide these in inches. It will go forward/back x many inches.
        """
        for module, position in zip(self.modules, positions):
            module.setModulePosition(position)

    def setUniformModulePosition(self, distance):
        """
        Sets a uniform distance for the drive motor to travel. Note,
        you should give the distance in inches. The modules will move this
        much in the direction they are facing.
        """
        for module in self.modules:
            module.setModulePosition(distance)

    # Cougar Course (2.0) Stuff Below

    def calculateCoefficientsCougarCourse(self, points: list):
        """
        So this builds upon the Bezier path stuff. Basically, the goal is
        to omit the need for control points. I have no idea if this is possible.
        But when has that ever stopped me from trying?
        Ideas:
        Look up "Parametric Cubic Spline Tutorial"
        """

        ts = self.generatePointPercentages(points)

        augmentedX, augmentedY = (
            [],
            [],
        )  # Create the augmented matrices. One for x and one for y because it's parametric.

        for t, point in zip(ts, points):
            subrowX = []
            for n in range(
                len(points) - 1, -1, -1
            ):  # Count backwards because order matters!
                subrowX.append(t ** n)  # Remember that n^0 is always 1!
            subrowX.append(point[0])  # Add the coordinate because it's augmented.
            augmentedX.append(subrowX)

            subrowY = []
            for n in range(
                len(points) - 1, -1, -1
            ):  # Count backwards because order matters!
                subrowY.append(t ** n)  # Remember that n^0 is always 1!
            subrowY.append(point[1])  # Add the coordinate because it's augmented.
            augmentedY.append(subrowY)

        rX = Matrix(augmentedX).rref()  # Calculate the coefficients vector for the Xs.
        rY = Matrix(augmentedY).rref()  # Calculate the coefficients vector for the Ys.

        return rX, rY  # Return the two vectors consisting of the coefficients.

    def setCoefficientsPosCougarCourse(self, xCoefficients: list, yCoefficients: list):
        """
        Sets the coefficients for the equations.
        Remember, we actually need the derivative!
        Returns a function that can calculate the
        position of the robot at t.
        args:
            xCoefficients: The coefficients for the x(t) polynomial.
            yCoefficients: The coefficients for the y(t) polynomial.
        returns:
            A function that takes in a t value and returns
            the position of the robot at that t.
        """

        # Returns a function that can calculate the angle of the robot at t.
        # Basically does (dY/dt)/(dX/dt) and converts the slope to an angle.
        # Idk why we need the +90, but we do lol.
        return lambda t: [
            sum(
                [
                    float(xCoefficients[i]) * t ** (len(xCoefficients) - (i + 1))
                    for i in range(len(xCoefficients))
                ]
            ),  # AAAAAHHHHHHHHHHHHHHH
            sum(
                [
                    float(yCoefficients[i]) * t ** (len(yCoefficients) - (i + 1))
                    for i in range(len(yCoefficients))
                ]
            ),  # AAAAAHHHHHHHHHHHHHHH
        ]

    def setCoefficientsSlopeCougarCourse(
        self, xCoefficients: list, yCoefficients: list
    ):
        """
        Sets the coefficients for the equations.
        Remember, we actually need the derivative!
        Returns a function that can calculate the
        angle of wheels of the robot at t.
        args:
            xCoefficients: The coefficients for the x(t) polynomial.
            yCoefficients: The coefficients for the y(t) polynomial.
        returns:
            A function that can calculate the wheel angles at t.
        """

        # Returns a function that can calculate the angle of the robot at t.
        # Basically does (dY/dt)/(dX/dt) and converts the slope to an angle.
        # Idk why we need the +90, but we do lol.
        return (
            lambda t: math.atan2(
                sum(
                    [
                        (len(yCoefficients) - (i + 1))
                        * float(yCoefficients[i])
                        * t ** (len(yCoefficients) - i - 2)
                        for i in range(
                            len(yCoefficients) - 1
                        )  # Subtract one because the code gets pissy when raising to a negative power.
                    ]
                ),  # AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHH
                sum(
                    [
                        (len(xCoefficients) - (i + 1))
                        * float(xCoefficients[i])
                        * t ** (len(xCoefficients) - i - 2)
                        for i in range(
                            len(xCoefficients) - 1
                        )  # Subtract one because the code gets pissy when raising to a negative power.
                    ]
                ),  # AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHH
            )
            * 180
            / math.pi
            + 90
        )  # But it works lol.

    def estimateLengthCougarCourse(
        self, xCoefficients: list, yCoefficients: list, iterations: int = 1000
    ):
        """
        This method is very, very similar to the one in the Bezier
        code. Basically divide the curve into a bunch of small, straight
        segments and sum the lengths of each segment. The total
        should be a close approximation to the length of the curve.
        args:
            xCoefficients: The coefficients for the x(t) polynomial.
            yCoefficients: The coefficients for the y(t) polynomial.
            iterations: The number of segments to divide the curve into.
        returns:
            The estimated length of the curve.
        """

        # Establish the total length variable and previous position variables.
        length = 0
        previousX = 0
        previousY = 0

        # The function which we will call to get the position.
        posFunc = self.setCoefficientsPosCougarCourse(xCoefficients, yCoefficients)

        # Iterate through each step, taking the length of each sum with Pythagorean Theorem.
        for i in range(iterations + 1):
            t = i / iterations

            # "positions" is (x,y).
            positions = posFunc(t)

            if i > 0:
                xDiff = positions[0] - previousX
                yDiff = positions[1] - previousY
                length += math.sqrt(xDiff ** 2 + yDiff ** 2)

            previousX = positions[0]
            previousY = positions[1]

        # Return the sum of the segments.
        return length

    def generatePointPercentages(self, points: list):
        """
        Used in parametricSplineGenerator. Calculates t values between 0 and
        1 for each point.
        args:
            points: A list of points to pass through.
        returns:
            The corresponding t values for each point.
        """

        t = []

        # Create objects that allows us to track the x and y more easily.
        positions = self.createPositionObjects(points)

        t.append(0)  # Start with t = 0.
        for i in range(len(positions) - 1):
            try:
                t.append(
                    positions[i] + positions[i + 1] + t[i]
                )  # Calculate the distance between two points and add the accumulated distance.
            except (IndexError):
                t.append(
                    positions[i] + positions[i + 1]
                )  # Must be the first one, so just add the distance.
            last = positions[i] + positions[i + 1] + t[i]

        for i in range(len(t)):
            t[i] = t[i] / last

        return t

    # Bezier Stuff Below

    def getQuadraticBezierPosition(self, p: list, t):
        """
        Returns the position in the quadratic Bezier curve, given
        the control points and the percentage through the
        curve. See https://math.stackexchange.com/questions/1360891/find-quadratic-bezier-curve-equation-based-on-its-control-points.
        """

        # Returns (x, y) @ % = t
        return (
            ((1 - t) ** 2 * p[0][0] + 2 * t * (1 - t) * p[1][0] + t ** 2 * p[2][0]),
            ((1 - t) ** 2 * p[0][1] + 2 * t * (1 - t) * p[1][1] + t ** 2 * p[2][1]),
        )

    def getCubicBezierPosition(self, p: list, t: float):
        """
        Returns the position in the cubic Bezier curve, given
        the control points and the percentage through the
        curve. NOTE: Doesn't seem to be working. Please use
        getHigherBezierPositionInstead instead.
        """

        # Returns (x, y) @ % = t
        return (
            (
                (1 - t) ** 3 * p[0][0]
                + 3 * (1 - t) ** 2 * p[1][0]
                + 3 * (1 - t) ** 2 * p[2][0]
                + t ** 3 * p[3][0]
            ),
            (
                (1 - t) ** 3 * p[0][1]
                + 3 * (1 - t) ** 2 * p[1][1]
                + 3 * (1 - t) ** 2 * p[2][1]
                + t ** 3 * p[3][1]
            ),
        )

    def getHigherBezierPosition(self, p: list, t: float):
        """
        Ok this math is going to kill the robot lol. Look here:
        https://en.wikipedia.org/wiki/B%C3%A9zier_curve#General_definition
        for the math behind what I'm about to write. Tested, it
        works! Look at test. Test can be found here:
        https://www.researchgate.net/figure/Quintic-trigonometric-Bezier-curve-with-a-b_fig2_318599090
        """

        # The for loop will act as our summation.
        # Start at one, end at our given number.
        xSum = 0
        ySum = 0

        # Don't subtrct one here so we can iterate through each point.
        for i in range(len(p)):
            x = p[i][0]
            y = p[i][1]

            # Binomial coefficient stuff here ('n' is the 'w'):
            # https://math.stackexchange.com/questions/1713706/what-does-2-values-vertically-arranged-in-parenthesis-in-an-equation-mean
            # Remember, 'n' is NOT number of points; instead, it's the degree. This means an 'n' of five has six points.
            n = len(p) - 1
            binomialCoefficient = math.factorial(n) / (
                math.factorial(i) * math.factorial(n - i)
            )

            xSum += binomialCoefficient * (1 - t) ** (n - i) * t ** i * x
            ySum += binomialCoefficient * (1 - t) ** (n - i) * t ** i * y

        return (xSum, ySum)

    def getQuadraticBezierSlope(self, p: list, t):
        """
        Returns the slope of the current position along
        a quadratic bezier curve, defined by three given control points.
        """

        # Define the given points.
        x0 = p[0][0]
        y0 = p[0][1]
        x1 = p[1][0]
        y1 = p[1][1]
        x2 = p[2][0]
        y2 = p[2][1]

        # 'a' is the x, 'b' is the y
        a0 = x0 - (x0 - x1) * t
        b0 = y0 - (y0 - y1) * t

        a1 = x1 - (x1 - x2) * t
        b1 = y1 - (y1 - y2) * t

        # Return the slope as (y, x) of the two points we just calculated.
        return ((b1 - b0), (a1 - a0))

    def getCubicBezierSlope(self, p: list, t):
        """
        Returns the slope of the current position along
        a cubic bezier curve, defined by four given control points.
        """

        # Define the given points.
        x0 = p[0][0]
        y0 = p[0][1]
        x1 = p[1][0]
        y1 = p[1][1]
        x2 = p[2][0]
        y2 = p[2][1]
        x3 = p[3][0]
        y3 = p[3][1]

        # 'a' is the x, 'b' is the y
        a0 = x0 - (x0 - x1) * t
        b0 = y0 - (y0 - y1) * t

        a1 = x1 - (x1 - x2) * t
        b1 = y1 - (y1 - y2) * t

        a2 = x2 - (x2 - x3) * t
        b2 = y2 - (y2 - y3) * t

        c0 = a0 - (a0 - a1) * t
        d0 = b0 - (b0 - b1) * t

        c1 = a1 - (a1 - a2) * t
        d1 = b1 - (b1 - b2) * t

        # Return the slope calculated using the previous points
        return ((d1 - d0), (c1 - c0))

    def getHigherBezierSlope(self, p: list, t):
        """
        The derivative of the equation for the points.
        Note, it will be in terms of t. To make it in terms of dx/dy, we
        have to do division; more info here:
        https://pages.mtu.edu/~shene/COURSES/cs3621/NOTES/spline/Bezier/bezier-der.html
        (lol this page is from my college!)
        """

        dxDt = 0
        dyDt = 0

        # The for loop will act as our summation again.
        for i in range(len(p) - 1):
            x = p[i][0]
            y = p[i][1]

            nextX = p[i + 1][0]
            nextY = p[i + 1][1]

            # View the position method for binomial coefficient info.
            n = len(p) - 2
            binomialCoefficient = math.factorial(n) / (
                math.factorial(i) * math.factorial(n - i)
            )

            # (n + 1) restores 'n's original value, the length of p.
            qX = (n + 1) * (nextX - x)
            qY = (n + 1) * (nextY - y)

            dxDt += binomialCoefficient * (1 - t) ** (n - i) * t ** i * qX
            dyDt += binomialCoefficient * (1 - t) ** (n - i) * t ** i * qY

        # According to parametric differentiation, we can do the following, and get dyDx.
        return (dyDt, dxDt)

    def getQuadraticBezierLength(self, p: list):
        """
        Returns the length of a Quadratic Bezier
        curve given via control points. Source:
        https://gist.github.com/tunght13488/6744e77c242cc7a94859.
        """

        # Divide the points into invidual lists of x's and y's.
        xs, ys = map(list, zip(*p))

        # Are all of the x's the same?
        if len(xs) == xs.count(xs[0]):
            length = 0
            initial = 0
            for y in list(enumerate(ys)):
                length += abs(y[1] - initial)
                initial = ys[y[0]]

            return length

        # Are all of the y's the same?
        elif len(ys) == ys.count(ys[0]):
            length = 0
            initial = 0
            for x in list(enumerate(xs)):
                length += abs(x[1] - initial)
                initial = xs[x[0]]

            return length

        positions = self.createPositionObjects(p)

        if len(positions) != 3:
            raise Exception("Bruh this is a quadratic. Three points!")

        pointOne, pointTwo, pointThree = positions[0], positions[1], positions[2]

        aX = pointOne.x - 2 * pointTwo.x + pointThree.x
        aY = pointOne.y - 2 * pointTwo.y + pointThree.y

        bX = 2 * pointTwo.x - 2 * pointOne.x
        bY = 2 * pointOne.y - 2 * pointTwo.y

        A = 4 * (aX ** 2 + aY ** 2)
        B = 4 * (aX * bX + aY * bY)
        C = bX ** 2 + bY ** 2

        SABC = 2 * math.sqrt(A + B + C)
        A2 = math.sqrt(A)
        A32 = 2 * A * A2
        C2 = 2 * math.sqrt(C)
        BA = B / A2

        return (
            A32 * SABC
            + A2 * B * (SABC - C2)
            + (4 * C * A - B * B) * math.log((2 * A2 + BA + SABC) / (BA + C2))
        ) / (4 * A32)

    def getHigherBezierLength(self, p: list, iterations: int = 1000):
        """
        Ok. So with cubic bezier, their is no closed-integral
        definition for the cubic Bezier length. I've done lot's
        of research, and it appears as there is an approximation.
        We can approximate it by adding individual line segments together.
        "iterations" will track how many divisions we make, and thus
        the precision. Read up here https://www.lemoda.net/maths/bezier-length/index.html.
        1000 iterations is pretty good for our purposes.
        """

        # Establish the total length variable and previous position.
        length = 0
        previousX = 0
        previousY = 0

        # Iterate through each step, taking the length of each sum with Pythagorean Theorem.
        for i in range(iterations + 1):
            t = i / iterations

            # "positions" is (x,y).
            positions = self.getHigherBezierPosition(p, t)

            if i > 0:
                xDiff = positions[0] - previousX
                yDiff = positions[1] - previousY
                length += math.sqrt(xDiff ** 2 + yDiff ** 2)

            previousX = positions[0]
            previousY = positions[1]

        # Return the sum of the segments.
        return length

    def createPositionObjects(self, points: list):
        """
        Creates Position objects using a given list
        of Xs and Ys. Returns that new list.
        """
        return [Position(coord[0], coord[1]) for coord in points]

    def setCruiseVelocity(self, slow=False):
        """
        Changes the motion magic's max cruise velocity.
        """
        for module in self.modules:
            module.setDriveCruiseVelocity(slow)

    def setVariableCruiseVelocity(self, speed):
        """
        Sets the cruise velocity to any speed.
        """
        for module in self.modules:
            module.setVariableDriveCruiseVelocity(speed)


class Position:
    """
    No, not that garbage by Ariane Grande. Stores
    An X and Y value for convenience.
    """

    def __init__(self, x, y):
        self.x = x
        self.y = y

    def __add__(self, other: object):
        """
        Replaces the typical '+' operator with a distance calculator
        that uses the simple distance formula.
        """
        return math.sqrt((self.x - other.x) ** 2 + (self.y - other.y) ** 2)
