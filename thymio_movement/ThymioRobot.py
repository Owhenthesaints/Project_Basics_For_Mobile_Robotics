import threading
import time

import numpy as np

from dependencies.AsyncClientInterface import AsyncClientInterface


class ThymioRobot():
    """
    :param np.array self._position: describes the initial position
    """
    movement_thread = None
    goal = None
    __stop_thread = False

    AsyncClient = AsyncClientInterface()
    __TURN_FORWARD_RATE_SPEED_CONSTANT = 150

    def __init__(self, init_position=None, init_angle: float = 0, threshold: float = 10, time_to_turn_const=0.0165,
                 secs_to_m=0.015):
        self.__CONVERSION_CONSTANT_TTT = time_to_turn_const
        if init_position is None:
            self.position = np.array([0, 0, init_angle])
        else:
            self.position = np.concatenate((np.array(init_position), np.array(init_angle)))
        self.__SECS_TO_M = secs_to_m
        self.interrupt = threading.Event()
        self.__THRESHOLD = threshold

    def initialise_movement_thread(self):
        self.movement_thread = threading.Thread(target=self.__movement_function)
        self.movement_thread.start()

    def set_new_goal(self, new_goal: list):
        if not self.movement_thread.isAlive():
            self.initialise_movement_thread()
        self.goal = np.array(new_goal)
        self.interrupt.set()

    def __movement_function(self):
        while True:
            while self.goal is not None and not self.__stop_thread:
                movement_vector = self.goal - self.position[0:2]
                if np.linalg.norm(movement_vector, ord=2) < self.__THRESHOLD:
                    continue
                x = movement_vector[0]
                y = movement_vector[1]
                theta = self.position[2]
                self.turn((np.arctan2(y, x) - theta) / (2 * np.pi))

    def turn(self, normalised_angle: float, right: bool = True, turn_rate: int = 150) -> None:
        if right:
            self.AsyncClient.set_motors(right_motor=-turn_rate, left_motor=turn_rate)
        else:
            self.AsyncClient.set_motors(right_motor=turn_rate, left_motor=-turn_rate)

        time.sleep(
            normalised_angle * self.__CONVERSION_CONSTANT_TTT / turn_rate * self.__TURN_FORWARD_RATE_SPEED_CONSTANT)
        self.AsyncClient.set_motors(0, 0)

    def go_forward(self, m: float, speed: int = 150):
        self.AsyncClient.set_motors(150, 150)
        time.sleep(m * self.__SECS_TO_M / speed * self.__TURN_FORWARD_RATE_SPEED_CONSTANT)

    def __del__(self):
        self.__stop_thread = True
        self.movement_thread.join()
