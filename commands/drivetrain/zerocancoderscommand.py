from commands2 import CommandBase

import robot


class ZeroCANCodersCommand(CommandBase):
    """
    Used to zero the CANCoders. Ensure all wheels are straight, then
    call this command.
    """

    def __init__(self, offsets=[-255.9375, -271.9, -41.8, -130.1]):
        super().__init__()
        """
        Please remember, you are not updating the CANCoders with values of 0.
        Instead, use the Phoenix Tuner to read the angle when the wheels are 
        straight.
        """

        self.addRequirements(robot.drivetrain)

        self.offsets = offsets

    def initialize(self):

        robot.drivetrain.updateCANCoders(self.offsets)

        print("my angles (zeroes)" + str(robot.drivetrain.getModuleAngles()))
