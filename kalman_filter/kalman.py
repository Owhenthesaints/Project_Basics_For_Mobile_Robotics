import numpy as np
import time
import cv2

from kalman_filter.EKF import ExtendedKalmanFilter
import dependencies.constants_robot as cst
# limit the angle to (-pi, pi) 
def convert_angle(angle):
    angle = angle % (2*np.pi)
    if angle >= np.pi:
        angle -= 2*np.pi
    return angle



def kalman_filter(kalman: ExtendedKalmanFilter(position=np.array([0, 0, 0])) = ExtendedKalmanFilter, pos_measure = None, speed = None, dt = None):

    has_vision = (pos_measure is not None )
    if dt is None:
        t = time.time()
        dt = t - kalman.last_t()
        kalman.update_t(t)
    kalman.recompute_F(dt)
    kalman.predict()
    kidnapping = False

    if(has_vision):
        kalman_est_pos = kalman.current_estimate()
        pos_est_old, angle_est_old = kalman_est_pos[0:2], kalman_est_pos[2]
        angle_diff_abs = abs(convert_angle(angle_est_old - pos_measure[2]))
        dist = np.sqrt(np.sum(np.square(pos_est_old - pos_measure[0:2])))

        if dist > cst.DIST_TRESHOLD or angle_diff_abs > cst.ANGLE_TRESHOLD:
            print("Detecting kidnapping")
            kalman.update_t(time.time())
            kalman.init_state_vector(pos_measure, speed)
            kidnapping = True

        else:
            kalman.update(has_vision, pos_measure, speed)
    else:
        kalman.update(has_vision, pos_measure, speed)

    kalman_est_update= kalman.current_estimate() # based on what the kalman has updated


    return kalman_est_update, kidnapping