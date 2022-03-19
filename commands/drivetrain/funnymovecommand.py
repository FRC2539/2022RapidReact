from commands2 import CommandBase
import robot


class FunnyMoveCommand(CommandBase):
    def __init__(
        self,
        distance,
        angle=0,
        tolerance=5,
        slow=False,
        torySlow=None,
        toryAcc=None,
        name=None,
    ):
        """
        I have actually gone crazy now
        """

        if name is None:
            name = "Move %f inches" % distance

        super().__init__()

        self.distance = -robot.drivetrain.metersToInches(distance)
        self.angle = angle
        self.tol = tolerance  # Angle tolerance in degrees.
        self.isSlow = slow
        self.torySlow = torySlow
        self.toryAcc = toryAcc

        self.moveSet = False
        self.addRequirements(robot.drivetrain)

    def initialize(self):
        if self.isSlow:
            robot.drivetrain.setCruiseVelocity(True)

        if self.torySlow != None:  # This IF overrides the previous one.
            robot.drivetrain.setVariableCruiseVelocity(
                self.torySlow if self.torySlow != "test" else robot.drivetrain.testSlow
            )

        if self.toryAcc != None:
            robot.drivetrain.setVariableCruiseAcceleration(
                self.toryAcc if self.toryAcc != "test" else robot.drivetrain.testAcc
            )

        robot.drivetrain.setModuleProfiles(1, turn=False)

        self.count = 0
        self.startPos = robot.drivetrain.getPositions()

        robot.drivetrain.setUniformModuleAngle(self.angle)

    def execute(self):
        self.count = 0
        print(robot.drivetrain.getModuleAngles())
        if self.count != 4 and not self.moveSet:
            for currentAngle in robot.drivetrain.getModuleAngles():
                if (
                    abs(currentAngle - self.angle) < self.tol
                    or abs(currentAngle - self.angle - 360) < self.tol
                ):
                    self.count += 1
                else:
                    continue

        if self.count == 4:  # All angles aligned.
            print("ALL ANGLES ALIGNED, SETTING POSITIONS")
            robot.drivetrain.setPositions(
                [self.distance, self.distance, self.distance, self.distance]
            )

            self.moveSet = True

        robot.drivetrain.setUniformModuleAngle(self.angle)

    def isFinished(self):
        count = 0
        for position, start in zip(robot.drivetrain.getPositions(), self.startPos):
            if abs(position - (start + self.distance)) < 4:
                count += 1
            else:
                print("not finished: " + str(abs(position - (start + self.distance))))
                return False

        if count == 4:
            return True

    def end(self, interrupted):
        robot.drivetrain.stop()
        robot.drivetrain.setCruiseVelocity()
        robot.drivetrain.setCruiseAcceleration()
        robot.drivetrain.setModuleProfiles(0, turn=False)
        self.moveSet = False
        print("\n\n\nDone\n\n\n")
