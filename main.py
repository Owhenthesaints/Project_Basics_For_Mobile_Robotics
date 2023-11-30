import numpy as np
import cv2

# import dependencies.constants_robot as constants_robot
# from dependencies.helper_functions import NoThymioError
# from kalman_filter.EKF import ExtendedKalmanFilter
# from kalman_filter.kalman import kalman_filter
# from thymio_movement.ThymioRobot import ThymioRobot
from global_navigation.GlobalNav2 import GlobalNav2

DO_KALMAN = True
LOCAL_AVOIDANCE = True
VISION = True

if __name__ == "__main__":
    # CAMERA INITIALISATION
    global_navigation = GlobalNav2()
    print("done")
    while True:
        successful_pos_update = False
        while not successful_pos_update:
            successful_pos_update = global_navigation.update_robot_pos_and_angle()
        rob_pos = global_navigation.get_robot_pos_and_angle()
        print(rob_pos)
        global_navigation.show_image(True, True, False)
        key = cv2.waitKey(20)
        if key == 27: # exit on ESC
            break



    # if DO_KALMAN:
    #     if position is None:
    #         raise NoThymioError("no thymio found for initialisation of Kalman Filter")
    #     KF = ExtendedKalmanFilter(position)

    # little_thymio = ThymioRobot()




    # #main Loop
    # while little_thymio.is_alive:



    #     if np.any(little_thymio.get_sensors() > constants_robot.LOCAL_NAV_SWITCH):
    #         little_thymio.local_nav()
    #     # DO_Kalman initialisation
    #     if VISION:

    #         position = None

    #     # apply kalman
    #     if DO_KALMAN:
    #         speed = little_thymio.get_sensors(sensor="wheels")
    #         position_update, kidnapped = kalman_filter(KF, position, speed)
    #         # set the new position as argument of set_new_position
    #         little_thymio.set_new_position(position_update)

    #     # here make final condition if thymio ends up on the final point
    #     if little_thymio.on_objective:
    #         little_thymio.kill()
