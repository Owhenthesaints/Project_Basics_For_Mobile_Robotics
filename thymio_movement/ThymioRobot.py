from dependencies.AsyncClientInterface import AsyncClientInterface


class ThymioRobot():

    def __init__(self, init_position=None):
        if init_position is None:
            init_position = [0, 0]
        self.asyncClient = AsyncClientInterface()
        self._position = init_position

