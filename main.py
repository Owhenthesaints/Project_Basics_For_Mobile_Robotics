import numpy as np
import time
import dependencies.constants_robot as constants_robot
from thymio_movement.ThymioRobot import ThymioRobot
from kalman_filter.kalman import kalman_filter
from kalman_filter.EKF import ExtendedKalmanFilter
import thymio_movement.local_navigation as local

DO_KALMAN = True
LOCAL_AVOIDANCE = True
NEED_INIT = True

goal_position = [1000, 1000, np.pi/2]
position = [0, 0, np.pi/2]


if __name__=="__main__":
    #initialize vision
    KF = ExtendedKalmanFilter()
    if DO_KALMAN:
        if position is None:
            KF.update_t(time.time())
            KF.init_state_vector(position, np.array([0, 0]))
            NEED_INIT = False
        else:
            print("Thymio has not been found")

    little_thymio = ThymioRobot()
    while little_thymio.is_alive:
        if np.any(little_thymio.get_sensors() > constants_robot.LOCAL_NAV_SWITCH):
             little_thymio.local_nav()
        # DO_Kalman initialisation

        # apply kalman
        if DO_KALMAN:
            speed = little_thymio.get_sensors(sensor="wheels")
            position_update, kidnapped = kalman_filter(KF, position, speed)
            # set the new position as argument of set_new_position
            little_thymio.set_new_position(position_update)

        # here make final condition if thymio ends up on the final point
        if little_thymio.on_objective:
            little_thymio.kill()


