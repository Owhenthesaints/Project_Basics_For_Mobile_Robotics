import cv2
import numpy as np

import dependencies.constants_robot as constants_robot
from global_navigation.GlobalNav2 import GlobalNav2
from kalman_filter.Filter_class import ExtendedKalmanFilter
from kalman_filter.Filter_func import kalman_func
from thymio_movement.ThymioRobot import ThymioRobot

DO_KALMAN = True
LOCAL_AVOIDANCE = True
VISION = True
MOVEMENT = True

if __name__ == "__main__":

    # CAMERA INITIALISATION
    global_navigation = GlobalNav2()
    print("done")

    ### add kalman filter ###################
    while not global_navigation.find_thymio():
        pass
    if DO_KALMAN:
        KF = ExtendedKalmanFilter(global_navigation.get_position_and_angle())
    ########################################
    little_thymio = ThymioRobot()

    while little_thymio.is_not_happy:

        if LOCAL_AVOIDANCE:
            while np.any(little_thymio.get_sensors() > constants_robot.LOCAL_NAV_SWITCH):
                little_thymio.apply_local_nav()
                if global_navigation.find_thymio():
                    global_navigation.append_position_to_history()

        if VISION:
            # waiting to find thymio position
            if DO_KALMAN:
                position_update = None
                if global_navigation.find_thymio():
                    speed = little_thymio.get_sensors(sensor="wheels")
                    print("before update :", global_navigation.get_position_and_angle())
                    position_update, position_variance, kidnapping = kalman_func(KF,
                                                                                 global_navigation.get_position_and_angle(),
                                                                                 speed)
                    print("after update", position_update)
                    if kidnapping:
                        little_thymio.angry()
                    # set the new position as argument of set_new_position
                else:
                    speed = little_thymio.get_sensors(sensor="wheels")
                    position_update, position_variance, kidnapping = kalman_func(KF, None, speed)
                    print("after update kalman no pos", position_update)
                    # set the new position as argument of set_new_position
                global_navigation.override_position(position_update)
                little_thymio.set_new_position(position_update)
            else:
                while not global_navigation.find_thymio():
                    pass
                little_thymio.set_new_position(global_navigation.get_position_and_angle())
            global_navigation.append_position_to_history()
            global_navigation.show_image(True, True, True, True, True, estimate=position_update,
                                         probability=position_variance)
            little_thymio.set_new_goal(global_navigation.get_next_position())

        ####### astolfi controller ##########
        if MOVEMENT:
            little_thymio.apply_motor_command()
        #####################################

        if global_navigation.get_on_goal():
            little_thymio.be_happy()

        key = cv2.waitKey(20)
        if key == 27:  # exit on ESC
            little_thymio.be_happy()
            break
