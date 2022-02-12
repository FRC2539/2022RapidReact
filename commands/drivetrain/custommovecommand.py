from commands2 import CommandBase

import robot

from wpimath.geometry import Transform2d
from wpimath.kinematics import ChassisSpeeds


class CustomMoveCommand(CommandBase):
    """
    Uses a non-wpilib algorithm to move to position relative to the robot.
    """

    def __init__(
        self,
        x=0,
        y=0,
        relative=True,
        flipDirection=False,
        maxSpeed=0.5,
        minSpeed=0.05,
        distanceTolerance=0.1,
        slowdownDistance=0.5,
    ):
        super().__init__()

        self.addRequirements(robot.drivetrain)

        # Correct the axes if it is chosen to flip them
        self.directionCorrection = -1 if flipDirection else 1

        # Store the arguments
        self.x = x * -1  # multiply by -1 for correcting the direction
        self.y = y * -1
        self.relative = relative
        self.maxSpeed = maxSpeed
        self.minSpeed = minSpeed
        self.distanceTolerance = distanceTolerance
        self.slowdownDistance = slowdownDistance

        self.wheelAngleTolerance = 2  # degrees

    def initialize(self):
        # Initialize the robot state
        self.initializeState()

        # Set the initial wheel angles
        robot.drivetrain.setModuleStates(self.initialModuleStates)

    def execute(self):
        # Wait for the wheel angles to face the correct direction
        if not self.moduleStatesMatch and not self.moduleStateAnglesMatch(
            self.initialModuleStates
        ):
            return

        # Calculate the distance to the target position
        self.distance = self.calculateDistanceToFinalPose()

        # Calculate the chassis speeds based on distance
        # (speed proportions are constants)
        chassisSpeeds = self.calculateSpeeds(self.distance)

        # Set the chassis speeds for the robot
        robot.drivetrain.setChassisSpeeds(chassisSpeeds)

    def isFinished(self):
        # Returns whether or not the robot has reached the target (distance based)
        return self.distance <= self.distanceTolerance

    def end(self, interrupted):
        robot.drivetrain.stop()

    def initializeState(self):
        # Store the initial location of the robot
        self.initialPosition = self.getRobotPose()

        # Calculate the final location of the robot
        # relative to the current location
        self.finalPosition = self.calculateFinalRobotPose()

        # Calculate the speeds needed in each axis as percents
        self.speedPercents = self.calculateAxisSpeedsAsPercents()

        # Calculate the distance to the final position and store it
        self.distance = self.calculateDistanceToFinalPose()

        # Calculate the wheel angles needed
        self.initialModuleStates = self.calculateInitialModuleStates()

        # Tracks if the modules are facing the correct angles
        self.moduleStatesMatch = False

    def calculateSpeeds(self, distance):
        speed = 0

        # Chooses a speed based on the distance to the target pose
        if distance <= self.slowdownDistance:
            speed = self.minSpeed
        else:
            speed = self.maxSpeed

        # Flip the speed direction if that was configured
        speed *= self.directionCorrection

        # Return field relative speeds in the correct direction
        return robot.drivetrain.getRobotRelativeSpeedsFromFieldSpeeds(
            self.speedPercents.vx * speed, self.speedPercents.vy * speed, 0
        )

    def calculateInitialModuleStates(self):
        # Calculate the initial speeds for the wheels
        chassisSpeeds = self.calculateSpeeds(self.distance)

        # Determine the necessary module states to achieve those speeds
        moduleStates = robot.drivetrain.convertChassisSpeedsToModuleStates(
            chassisSpeeds
        )

        optimizedModuleStates = robot.drivetrain.optimizeModuleStates(moduleStates)

        # Zero the speeds of the modules, but keep the angles
        for moduleState in optimizedModuleStates:
            moduleState.speed = 0

        return optimizedModuleStates

    def moduleStateAnglesMatch(self, targetModuleStates):
        currentModuleStates = robot.drivetrain.getModuleStates()

        match = True

        # Check if any of the modules are not facing the correct angle
        for i in range(0, len(targetModuleStates)):
            currentAngle = currentModuleStates[i].angle.degrees()
            targetAngle = targetModuleStates[i].angle.degrees()

            angleDistance = abs(currentAngle - targetAngle)

            # Check if the wheel is at the correct angle
            if not (
                angleDistance <= self.wheelAngleTolerance
                or angleDistance >= (360 - self.wheelAngleTolerance)
            ):  # degree tolerance
                match = False

        # Update the state variable
        self.moduleStatesMatch = match

        return match

    def calculateDistanceToFinalPose(self):
        return (
            self.getRobotPose().translation().distance(self.finalPosition.translation())
        )

    def calculateAxisSpeedsAsPercents(self):
        # Calculate the change in x, y, and (angle) to get to the final pose
        twist = self.initialPosition.log(self.finalPosition)

        maxVelocity = max(abs(twist.dx), abs(twist.dy))

        # Prevent a potential division by zero exception
        if maxVelocity == 0:
            return ChassisSpeeds(0, 0, 0)

        # Return the normalized (desaturated) wheel speeds
        return ChassisSpeeds(twist.dx / maxVelocity, twist.dy / maxVelocity, 0)

    def calculateFinalRobotPose(self):
        transform = Transform2d(self.x, self.y, 0)

        return self.initialPosition.transformBy(transform)

    def getRobotPose(self):
        return robot.drivetrain.getSwervePose()
