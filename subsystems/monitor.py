from .cougarsystem import CougarSystem


class Monitor(CougarSystem):
    """Exists to observe system state via its default command."""

    def __init__(self):
        super().__init__()

    def initDefaultCommand(self):
        from commands.monitorcommand import MonitorCommand

        self.setDefaultCommand(MonitorCommand())
