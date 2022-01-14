from .cougarsystem import CougarSystem
import ports
import math


class Climber(CougarSystem):
    """Controls the hardware of the climber mechanism."""

    def __init__(self):
        super().__init__("Climber")

    def periodic(self):
        """
        Loops when nothing else is running in
        this subsystem. Do not call this!
        """
        self.feed()
