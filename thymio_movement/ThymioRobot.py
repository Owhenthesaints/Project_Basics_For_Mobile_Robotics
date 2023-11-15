from dependencies.AsyncClientInterface import AsyncClientInterface


class ThymioRobot():

    def __init__(self, init_position: list[int, int]= [0, 0]):
        self.asyncClient = AsyncClientInterface()
        self._position = init_position

