import time
import threading
import numpy as np

from dependencies.AsyncClientInterface import AsyncClientInterface


class ThymioRobot():
    """
    :param np.array self._position: describes the initial position
    """

    AsyncClient = AsyncClientInterface()
    __TURN_FORWARD_RATE_SPEED_CONSTANT = 150
    def __init__(self, init_position=None, init_angle: float = 0, time_to_turn_const=0.0165, secs_to_m = 0.015):
        self.__CONVERSION_CONSTANT_TTT = time_to_turn_const
        if init_position is None:
            self._position = np.array([0, 0, init_angle])
        else:
            self._position = np.concatenate(np.array(init_position), np.array(init_angle))
        self.__SECS_TO_M = secs_to_m

    def __movement_function(self, position_to_go):
        movement_vector = position_to_go - self._position[0:2]
        x = movement_vector[0]
        y = movement_vector[1]
        theta = movement_vector[2]
        self.turn(np.arctan2(y, x) - theta)

    def turn(self, degrees: float, right: bool = True, turn_rate: int = 150) -> None:
        if right:
            self.AsyncClient.set_motors(right_motor=-turn_rate, left_motor=turn_rate)
        else:
            self.AsyncClient.set_motors(right_motor=turn_rate, left_motor=-turn_rate)

        time.sleep(degrees * self.__CONVERSION_CONSTANT_TTT / turn_rate * self.__TURN_FORWARD_RATE_SPEED_CONSTANT)
        self.AsyncClient.set_motors(0, 0)

    def go_forward(self, m: float, speed: int = 150):
        self.AsyncClient.set_motors(150, 150)
        time.sleep(m*self.__SECS_TO_M/speed*self.__TURN_FORWARD_RATE_SPEED_CONSTANT)
