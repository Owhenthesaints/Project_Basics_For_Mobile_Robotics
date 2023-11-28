import numpy as np

import dependencies.constants_robot as constants_robot
from dependencies.helper_functions import NoThymioError
from kalman_filter.EKF import ExtendedKalmanFilter
from kalman_filter.kalman import kalman_filter
from thymio_movement.ThymioRobot import ThymioRobot
from global_navigation.GlobalNav2 import *

DO_KALMAN = True
LOCAL_AVOIDANCE = True
VISION = True

if __name__ == "__main__":
    # CAMERA INITIALISATION
    CAMERA_ID = 1
    video_stream, QR_detector = init_camera_QRdetector(1)
    obstacle_vertices, goal_center, transformation_matrix = init_background(video_stream)
    recalculate_global = True
    last_robot_center = None
    windowName = "Tracking"

    # initialize vision
    position = [0, 0, np.pi / 2]

    if DO_KALMAN:
        if position is None:
            raise NoThymioError("no thymio found for initialisation of Kalman Filter")
        KF = ExtendedKalmanFilter(position)

    little_thymio = ThymioRobot()




    #main Loop
    while little_thymio.is_alive:



        if np.any(little_thymio.get_sensors() > constants_robot.LOCAL_NAV_SWITCH):
            little_thymio.local_nav()
        # DO_Kalman initialisation
        if VISION:

            position = None

        # apply kalman
        if DO_KALMAN:
            speed = little_thymio.get_sensors(sensor="wheels")
            position_update, kidnapped = kalman_filter(KF, position, speed)
            # set the new position as argument of set_new_position
            little_thymio.set_new_position(position_update)

        # here make final condition if thymio ends up on the final point
        if little_thymio.on_objective:
            little_thymio.kill()
