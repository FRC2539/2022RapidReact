from commands2 import WaitCommand


class SurrogateShooterCommand(WaitCommand):
    """
    Essentially a standin for shooting while we don't have a shooter
    """

    def __init__(self, delay=2):
        super().__init__(delay)
