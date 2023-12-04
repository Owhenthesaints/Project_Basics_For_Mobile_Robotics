import numpy as np
import cv2
import dependencies.constants_robot as constants_robot
# from dependencies.helper_functions import NoThymioError
from kalman_filter.EKF import ExtendedKalmanFilter
from kalman_filter.kalman import kalman_filter
from thymio_movement.ThymioRobot import ThymioRobot
from global_navigation.GlobalNav2 import GlobalNav2

DO_KALMAN = False
LOCAL_AVOIDANCE = False
VISION = True
MOVEMENT = True

if __name__ == "__main__":

    # CAMERA INITIALISATION
    global_navigation = GlobalNav2()
    print("done")
    
    ### add kalman filter ###################
    if DO_KALMAN:
        KF = ExtendedKalmanFilter(rob_pos)
    ########################################
    little_thymio = ThymioRobot()
    
    #### TESTING
    
    while not global_navigation.find_thymio():
        pass
    position = global_navigation.get_position_and_angle()
    goal = np.array([position[0]-200, position[1], np.pi/2])
    little_thymio.set_new_goal(goal)
    
    #### END TESTING
    
    while little_thymio.is_alive:

        if LOCAL_AVOIDANCE:
            if np.any(little_thymio.get_sensors() > constants_robot.LOCAL_NAV_SWITCH):
                little_thymio.local_nav()

        if VISION:
            # waiting to find thymio position
            while not global_navigation.find_thymio():
                pass
            global_navigation.show_image(True, True, False)
            curr_position = (global_navigation.get_position_and_angle())
            print("current position:", curr_position)
            little_thymio.set_new_position(curr_position)

        if DO_KALMAN:
            speed = little_thymio.get_sensors(sensor="wheels")
            position_update, kidnapped = kalman_filter(KF, rob_pos, speed)
            # set the new position as argument of set_new_position
            little_thymio.set_new_position(position_update)

        # [To Do]
        # 1. now the GloablNav2 class decide on_objective only by position from vision -> need to change to position from kalman filter 
        # possible solution: add a new function in GlobalNav2 class to get position from kalman filter (also the probality for drawing)
        #
        # 2. decide when to go to the next goal in shortest_path list
        # possible solution: add a new function in GlobalNav2 class to get the distance between robot and current goal
        #
        # 3. shortest_path list is in GlobalNav2 class -> need to change give the list to ThymioRobot class for astolfi controller
        # possible solution: add a new function in GlobalNav2 class to get the shortest_path list and then use set_new_goal function in ThymioRobot class
        #
        # 4. draw trajectory in the image
        #
        # 5. show kalman filter  probability in the image


        ####### astolfi controller ##########
        if MOVEMENT:
            little_thymio.apply_motor_command()
        #####################################


        if little_thymio.on_objective:
            little_thymio.kill()

        key = cv2.waitKey(20)
        if key == 27: # exit on ESC
            break



