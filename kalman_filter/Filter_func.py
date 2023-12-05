import numpy as np
from kalman_filter.Filter_class import ExtendedKalmanFilter
from dependencies.helper_functions import convert_angle
import dependencies.constants_robot as cst

def kalman_func(kalman: ExtendedKalmanFilter(position=np.array([0, 0, 0])) = ExtendedKalmanFilter, position = None, wheelspeed = None):

    kalman.get_dt()
    kalman.set_state_transition_matrix()
    kalman.predict()
    kidnapping = False
    
    # check if the robot has vision
    if position is not None:
        measurement = np.array([position[0], position[1], position[2], wheelspeed[0], wheelspeed[1]])
        kalman_est_pos, kalman_variance = kalman.update_vision(measurement)
        distance = np.linalg.norm(kalman_est_pos[0:2] - position[0:2], 2)
        angle_diff = convert_angle(kalman_est_pos[2] - position[2])
        if distance > cst.DIST_TRESHOLD or abs(angle_diff) > cst.ANGLE_TRESHOLD:
            print("Detecting kidnapping")
            kalman.get_dt()
            kidnapping = True
            kalman.init_state_vector(position, wheelspeed)
    else:
        kalman_est_pos, kalman_variance = kalman.update_encoder(wheelspeed)
        print("No vision, only encoder")

    return kalman_est_pos, kalman_variance, kidnapping        

