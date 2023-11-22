import numpy as np

import dependencies.constants_robot as constants_robot
from dependencies.AsyncClientInterface import AsyncClientInterface


class ThymioRobot():
    """
    :param np.array self.position: describes the initial position and the angle in the clockwise direction
    """
    movement_thread = None
    goal = None
    __WHEEL_DISTANCE = 0.09  # in m
    AsyncClient = AsyncClientInterface()
    on_objective: bool = False

    def __init__(self, init_position=None,
                 kappa_alpha: float = constants_robot.DEFAULT_KAPPA_ALPHA,
                 kappa_rho: float = constants_robot.DEFAULT_KAPPA_RHO,
                 kappa_beta: float = constants_robot.DEFAULT_KAPPA_BETA,
                 threshold: float = constants_robot.DEFAULT_THRESHOLD,
                 secs_to_m=0.015):
        if init_position is None:
            self._position = np.array([0, 0, 0])
        else:
            self._position = init_position
        self.__SECS_TO_M = secs_to_m
        self.__THRESHOLD = threshold
        self.__KAPPA_ALPHA = kappa_alpha
        self.__KAPPA_BETA = kappa_beta
        self.__KAPPA_RHO = kappa_rho

    def set_new_goal(self, new_goal: list[int, int, int] | np.ndarray):
        self.goal = np.array(new_goal)

    def set_new_position(self, position: np.array):
        self._position = position

    def stop(self):
        self.AsyncClient.set_motors(left_motor=0, right_motor=0)

    def get_sensors(self, sensor: str = "horizontal_sensor") -> list | np.ndarray | int:
        """
        :param str sensor: either "horizontal_sensor" one gets classical horizontal sensors or "wheels" one gets left
        then right wheel speed "wheels_target" the value stored in aseba.
        :rtype np.ndarray:
        :return: left then right wheel or horizontal array of sensors
        """
        if sensor == "horizontal_sensor":
            return np.array(self.AsyncClient.get_sensor(self.AsyncClient.PROX_HORIZONTAL_VALUES))
        elif sensor == "wheels":
            return np.array([self.AsyncClient.get_sensor(self.AsyncClient.LEFT_SPEED),
                             self.AsyncClient.get_sensor(self.AsyncClient.RIGHT_SPEED)])

    def apply_motor_command(self):
        movement_vector = self._position - self.goal
        rho = np.sqrt(movement_vector[1] ** 2 + movement_vector[0] ** 2)
        if rho < self.__THRESHOLD:
            self.on_objective = True
            return
        alpha = -self._position[2] - self.goal[2]
        beta = - self.goal[2]
        forward_speed = self.__KAPPA_RHO * rho
        turning_velocity = self.__WHEEL_DISTANCE / 2 * (self.__KAPPA_ALPHA * alpha + self.__KAPPA_BETA * beta)
        movement_array = [-turning_velocity + forward_speed, turning_velocity + forward_speed]
        print(movement_array[0], movement_array[1])
        self.AsyncClient.set_motors(left_motor=int(np.floor(movement_array[0])),
                                    right_motor=int(np.floor(movement_array[1])))
        self.on_objective = False
