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

def main():
    # initialise vision

    # initialise kalman
    KF = ExtendedKalmanFilter()
    # initialise state
    # 1 is global navigation, 2 is local avoidance
    state = 1 
    # initialisation of the thymio
    little_thymio = ThymioRobot()
    while little_thymio.is_alive:
        # if np.any(little_thymio.get_sensors() > constants_robot.LOCAL_NAV_SWITCH):
        #     little_thymio.local_nav()
        # detect position of thymio visually
        position = ...

        # apply kallman
        if DO_KALMAN and NEED_INIT:
            if position is None:
                KF.update_t(time.time())
                KF.init_state_vector(position, np.array([0, 0]))
                NEED_INIT = False
            else:
                print("Thymio has not been found")
                continue

        if DO_KALMAN:
            speed = little_thymio.get_sensors(sensor="wheels")
            position_upadte, kidnapped = kalman_filter(KF, position, speed)
            # set the new position as argument of set_new_position
            little_thymio.set_new_position(position_upadte)

        if LOCAL_AVOIDANCE:
            obstacles = little_thymio.get_sensors(sensor="horizontal_sensor")
            new_state = local.update_state(state, obstacles)
            if new_state != state:
                print(f"State has changed from {state} to {new_state}")
                state = new_state

        if little_thymio.on_objective:
            # then give it the next node/objective
            little_thymio.set_new_goal()
        if kidnapped:
            # here do steps to find new_goal
            little_thymio.set_new_goal()

        if state == 1:
            # astolfi command
            little_thymio.apply_motor_command()
        else:
            print("Local avoidance state")
            little_thymio.local_nav()
        

        # here make final condition if thymio ends up on the final point
        if ...:
            little_thymio.kill()


if __name__ == "__main__":
    main()
