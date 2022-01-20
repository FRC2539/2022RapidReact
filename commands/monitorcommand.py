from commands2 import CommandBase

import robot


class MonitorCommand(CommandBase):
    """Runs continually while the robot is powered on."""

    def __init__(self):
        super().__init__()

        """
        Required because this is the default command for the monitor subsystem.
        """
        # self.addRequirements(robot.monitor)

        self.setInterruptible(False)
        self.setRunWhenDisabled(True)

    def execute(self):
        """Implement watchers here."""
        pass
