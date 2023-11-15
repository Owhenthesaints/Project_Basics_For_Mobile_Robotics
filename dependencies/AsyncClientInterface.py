# I asked and anybody can use this file just keep
# Owhenthesaints/AsyncClinetInterface
import threading
import time
from typing import Callable, Any

from tdmclient import ClientAsync, aw


class InvalidArgumentError(Exception):
    pass


class AsyncClientInterface:
    """
    This is a class aimed to make the handling of the node easier and more readable
    """
    LEFT_MOTOR = "motor.left.target"
    RIGHT_MOTOR = "motor.right.target"
    PROX_HORIZONTAL_VALUES = "prox.horizontal"
    LEDS_BOTTOM_LEFT = "leds.bottom.left"
    LEDS_BOTTOM_RIGHT = "leds.bottom.right"
    PROX_GROUND_DELTA = "prox.ground.delta"
    PROX_GROUND_REFLECTED = "prox.ground.reflected"
    MIC_INTENSITY = "mic.intensity"
    LEDS_TOP = "leds.top"
    __right_motor_value: int = 0
    __left_motor_value: int = 0
    _delta_calib = 1
    _refl_calib = 1

    def __init__(self, delta_calib: float = None, refl_calib: float = None, time_to_turn_const: float = 1.36):
        """
        :param delta_calib: input known calibration for me it is 1.35
        :param refl_calib: input known calibration for me it is also about 1.35
        :param time_to_turn_const: time in seconds to turn 360°
        :type time_to_turn_const: float
        """
        self.client = ClientAsync()
        self.node = aw(self.client.wait_for_node())
        aw(self.node.lock())
        self.__CONVERSION_CONSTANT_TTT = time_to_turn_const
        if refl_calib is not None:
            self._refl_calib = refl_calib
        if delta_calib is not None:
            self._delta_calib = delta_calib
        if delta_calib is None and refl_calib is None:
            self.calibrate_sensor_prox_ref(True, True)
            self.calibrate_sensor_prox_ref(False, True)

    def sleep(self, seconds):
        aw(self.client.sleep(seconds))

    def get_refl_calib(self) -> float:
        return self._refl_calib

    def get_del_calib(self) -> float:
        return self._delta_calib

    def set_motors(self, *args, **kwargs) -> None:
        """
        A flexible function that lets you input the speed of the motors

        :param args: left_motor then right_motor
        :type args: int
        :param kwargs: can put in right_motor or left_motor value in directly or put in array with length 2
        :type kwargs: int or list[int]

        :return: None
        """
        if len(args) == 2:
            aw(self.node.set_variables({
                self.LEFT_MOTOR: [args[0]],
                self.RIGHT_MOTOR: [args[1]]
            }))

        elif len(args) == 1:
            aw(self.node.set_variables({
                self.LEFT_MOTOR: [args[0]]
            }))

        if 'left_motor' in kwargs:
            if isinstance(kwargs['left_motor'], int):
                aw(self.node.set_variables({
                    self.LEFT_MOTOR: [kwargs['left_motor']]
                }))
            else:
                raise TypeError("value attributed to left_motor was not int")

        if 'right_motor' in kwargs:
            if isinstance(kwargs['right_motor'], int):
                aw(self.node.set_variables({
                    self.RIGHT_MOTOR: [kwargs['right_motor']]
                }))
            else:
                raise TypeError("value attributed to right_motor was not int")
        if 'motor' in kwargs:
            if isinstance(kwargs['motor'], list) and len(kwargs['motor']) == 2:
                aw(self.node.set_variables({
                    self.LEFT_MOTOR: [int(kwargs['motor'][0])],
                    self.RIGHT_MOTOR: [int(kwargs['motor'][1])]
                }))
            else:
                raise TypeError("either array too long or too short or did not respect type list")

    def calibrate_sensor_prox_ref(self, delta: bool, set_val: bool = True, preset_Val: float = None) -> float | bool:
        """
            A function that returns an int (sensor_1/sensor_2) used to calibrate the two sensors of
            prox_ground_reflected

            :param set_val: defines whether you would like to set the internal calib val
            :type set_val: bool

            :param delta: if function should be used for delta or reflected
            :type delta: bool

            :returns: ref_sensor_1/ref_sensor_2
            :rtype: float or bool
        """

        ref_value = self.get_sensor("prox_del") if delta else self.get_sensor("prox_ref")
        calibration = ref_value[0] / ref_value[1]
        if preset_Val:
            if delta:
                self._delta_calib = preset_Val
            else:
                self._refl_calib = preset_Val
            return preset_Val
        elif ref_value:
            if set_val:
                if delta:
                    self._delta_calib = calibration
                else:
                    self._refl_calib = calibration
            return calibration
        else:
            return False

    def set_sensor(self, sensor: str, value: int | list[int]) -> None:
        """
        A function that enables you to set sensors through the node.set_variables method the leds expect values between 0 and 32
        :param sensor: input your sensor
            - 'leds_top': expects three rgb values
            - 'leds_bottom_left': expects r, g, b values
            - 'leds_bottom_right': expects r, g, b values
            - 'leds_circle': expects 8 values 1 or 0

        :param value: a list or an array containing the values you want to output
        """

        if sensor == "leds_top":
            if isinstance(value, list) and len(value) == 3:
                aw(self.node.wait_for_variables({"leds.top"}))
                for index, rgb in enumerate(value):
                    list(self.node.v.leds.top)[index] = rgb
            else:
                raise TypeError("either did not input list or list was not of right length")

    def get_sensor(self, sensor: str, calibrated: bool = False) -> list[int] | int | bool:
        """
        This is a function that returns the value of the sensor it accepts only a string and uses a switch case method


        :param sensor : Accepts  a limited number of strings\
            - 'prox_del' : this presents the array of prox ground delta with the option of calibration\
            - 'prox_ref' : this presents the array of prox ground reflected with the option of calibration\
            - self.CONSTANT: this is presented in the example in the description

        :type sensor: str

        :return: returns the value or array of a sensor, or returns False if invalid string is passed as argument
        :rtype: list[int] or int or bool or list[bool]

        Example:

        >>>Client = AsyncClientInterface(1.35,1.35)

        >>>print(Client.get_sensor(Client.PROX_GROUND_DELTA))
        """
        if sensor == "prox_del":
            aw(self.node.wait_for_variables({self.PROX_GROUND_DELTA}))
            if calibrated:
                calibrated_sensor_2 = int(list(self.node.v.prox.ground.delta)[1]) * self._delta_calib
                return [int(list(self.node.v.prox.ground.delta)[0]), int(calibrated_sensor_2)].copy()
            else:
                return list(self.node.v.prox.ground.delta).copy()

        elif sensor == "prox_ref":
            aw(self.node.wait_for_variables({self.PROX_GROUND_REFLECTED}))
            if calibrated:
                calibrated_sensor_2 = int(list(self.node.v.prox.ground.reflected)[1]) * self._refl_calib
                return [int(list(self.node.v.prox.ground.reflected)[0]), int(calibrated_sensor_2)].copy()
            else:
                return list(self.node.v.prox.ground.reflected).copy()

        else:
            aw(self.node.wait_for_variables({sensor}))
            attributes = sensor.split(".")
            node_with_attr = getattr(self.node, "v")
            for attribute in attributes:
                node_with_attr = getattr(node_with_attr, attribute)
            try:
                return list(node_with_attr)
            except AttributeError:
                raise InvalidArgumentError("attribute was not found due to wrong input being put in")

    def add_event_listener(self, function: Callable[[Any, Any, Any], None], emit: list[tuple[str, int]],
                           listening_time: int):
        """
        adds an event listener that will stop the program execution and wait to listen, and if a function inside the
        thymio emits then there will be a call to the function. The calls stack

        :param function: function you want to call
        :type function: Callable

        :param emit: emit takes the shape of the vars you want to read in your callable function
        :type emit: list[tuple[str,int]]

        :rtype: None


        """
        self.client.add_event_received_listener(function)

        async def prog():
            error = await self.node.register_events(emit)
            if error is not None:
                print(error)
            else:
                await self.node.watch(events=True)
            await self.client.sleep(listening_time)

    def add_listener(self, function: Callable[[..., dict], None], sleep_duration: int = None) -> None:
        """
        :param function: The event listener which should have
        :param sleep_duration: defines the sleep duration
        :return: None
        """

        async def prog():
            aw(self.node.watch(variables=True))
            self.node.add_variables_changed_listener(function)
            aw(self.client.sleep())

        self.client.run_async_program(prog)

    def turn(self, degrees: float, right: bool, ) -> None:
        def turn():
            if right:
                self.set_motors(left_motor=-50, right_motor=50)
            else:
                self.set_motors(left_motor=50, right_motor=-50)

            time.sleep(degrees * self.__CONVERSION_CONSTANT_TTT)
            self.set_motors(0, 0)

        turn.turn_thread = threading.Thread(target=turn)
        if turn.turn_thread.isAlive():
            turn.turn_thread.join()
        turn.turn_thread.start()

    def __del__(self):
        aw(self.node.unlock())
