import numpy as np
import time
import cv2

from EKF import ExtendedKalmanFilter

# thresholds for deciding kidnapping
DIST_TRESHOLD = 100
ANGLE_TRESHOLD = np.pi/4


# limit the angle to (-pi, pi) 
def convert_angle(angle):
    angle = angle % (2*np.pi)
    if angle >= np.pi:
        angle -= 2*np.pi
    return angle

def kalman_filter(kalman = ExtendedKalmanFilter, pos_sensor = None, angle_sensor = None, speed = None):

    has_vision = (pos_sensor is not None and angle_sensor is not None)
    t = time.time()
    dt = t - kalman.last_t()
    kalman.update_t(t)
    kalman.recompute_F(dt)
    kalman.predict()
    kidnapping = False

    if(has_vision):
        kalman_est_pos = kalman.current_estimate()
        pos_est_old, angle_est_old = kalman_est_pos[0:2], kalman_est_pos[2]
        angle_diff_abs = abs(convert_angle(angle_est_old - angle_sensor))
        dist = np.sqrt(np.sum(np.square(pos_est_old - pos_sensor)))

        if dist > DIST_TRESHOLD or angle_diff_abs > ANGLE_TRESHOLD:
            print("Detecting kidnapping")
            kalman.update_timestamp(time.time())
            kalman.init_state_vector(pos_sensor, angle_sensor, speed)
            kidnapping = True

        else:
            kalman.update(has_vision, pos_sensor, angle_sensor, speed)
    else:
        kalman.update(has_vision, pos_sensor, angle_sensor, speed)

    kalman_est_update= kalman.current_estimate() # based on what the kalman has updated
    pos_est_new = kalman_est_update[0:2]
    angle_est_new = kalman_est_update[2]

    return pos_est_new, angle_est_new, kidnapping