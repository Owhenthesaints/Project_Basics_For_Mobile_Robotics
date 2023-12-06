import numpy as np

import dependencies.constants_robot as constants_robot
from dependencies.AsyncClientInterface import AsyncClientInterface
from dependencies.helper_functions import convert_angle


class ThymioRobot():
    """
    :param np.array self.position: describes the initial position and the angle in the clockwise direction
    """
    movement_thread = None
    goal = None
    __WHEEL_DISTANCE = 0.09  # in m
    AsyncClient = AsyncClientInterface()
    on_objective: bool = False
    is_alive = True

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

    def kill(self):
        self.is_alive = False
        self.AsyncClient.set_motors(0, 0)

    def stop(self):
        self.AsyncClient.set_motors(left_motor=0, right_motor=0)

    def apply_local_nav(self):
        # first row left motor second right
        W = np.array([[1, 1, 1, -1, -1], [-1, -1, -1, 1, 1]]) * constants_robot.LOCAL_NAV_FACTOR
        lr_motor = W @ self.get_sensors().T + constants_robot.LOCAL_NAV_FORWARD_SPEED
        self.AsyncClient.set_motors(left_motor=int(lr_motor[0]), right_motor=int(lr_motor[1]))

    def get_sensors(self, sensor: str = "horizontal_sensor") -> list | np.ndarray | int:
        """
        :param str sensor: either "horizontal_sensor" one gets classical horizontal sensors or "wheels" one gets left
        then right wheel speed "wheels_target" the value stored in aseba.
        :rtype np.ndarray:
        :return: left then right wheel or horizontal array of sensors
        """
        if sensor == "horizontal_sensor":
            return np.array(self.AsyncClient.get_sensor(self.AsyncClient.PROX_HORIZONTAL_VALUES))[0:5]
        elif sensor == "wheels":
            return np.array([self.AsyncClient.get_sensor(self.AsyncClient.LEFT_SPEED),
                             self.AsyncClient.get_sensor(self.AsyncClient.RIGHT_SPEED)])
        
    def not_angry(self):
        self.AsyncClient.set_led("leds_top", [0, 0, 0])
        
    def angry(self):
        self.AsyncClient.set_led("leds_top", [32, 0, 0])

    def apply_motor_command(self):
        movement_vector = self.goal - self._position
        rho = np.sqrt(movement_vector[1] ** 2 + movement_vector[0] ** 2)
        if rho < self.__THRESHOLD:
            self.on_objective = True
            return
        # alpha = convert_angle(-self._position[2] + convert_angle(np.arctan2(movement_vector[1],movement_vector[0]))+np.pi)
        # beta = convert_angle(- self._position[2] - alpha - np.pi)
        # we add np.pi to position[2] in order to adapt to alstofy referential
        alpha = convert_angle(-self._position[2] + np.pi + np.arctan2(-movement_vector[1],movement_vector[0]))
        beta = convert_angle(- self._position[2] - alpha+ np.pi)
        forward_speed = self.__KAPPA_RHO * rho
        turning_velocity = -(self.__WHEEL_DISTANCE / 2) * (self.__KAPPA_ALPHA * alpha + self.__KAPPA_BETA * beta)
        if forward_speed > 150:
            turning_velocity *= 150 / forward_speed
            forward_speed = 150
        
        movement_array = [-turning_velocity + forward_speed, turning_velocity + forward_speed]
        self.AsyncClient.set_motors(left_motor=int(np.floor(movement_array[1])),
                                   right_motor=int(np.floor(movement_array[0])))
        self.on_objective = False
