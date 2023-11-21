import numpy as np

from dependencies.AsyncClientInterface import AsyncClientInterface


class ThymioRobot():
    """
    :param np.array self.position: describes the initial position and the angle in the clockwise direction
    """
    movement_thread = None
    goal = None
    __stop_thread = False
    __WHEEL_DISTANCE = 0.09  # in m
    AsyncClient = AsyncClientInterface()
    __TURN_FORWARD_RATE_SPEED_CONSTANT = 150
    on_objective: bool = False

    def __init__(self, init_position=None, init_angle: float = 0, KAPPA_ALPHA: float = 1., KAPPA_RHO: float = 1.,
                 KAPPA_BETA: float = 1.0, threshold: float = 10, time_to_turn_const=0.0165,
                 secs_to_m=0.015):
        self.__CONVERSION_CONSTANT_TTT = time_to_turn_const
        if init_position is None:
            self._position = np.array([0, 0, init_angle])
        else:
            self._position = np.concatenate((np.array(init_position), np.array(init_angle)))
        self.__SECS_TO_M = secs_to_m
        self.__THRESHOLD = threshold
        self.__KAPPA_ALPHA = KAPPA_ALPHA
        self.__KAPPA_BETA = KAPPA_BETA
        self.__KAPPA_RHO = KAPPA_RHO

    def set_new_goal(self, new_goal: list[int, int, int] | np.array):
        self.goal = np.array(new_goal)

    def set_new_position(self, position: np.array):
        self._position=position

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
        self.AsyncClient.set_motors(left_motor=movement_array[0], right_motor=movement_array[1])
        self.on_objective = False
