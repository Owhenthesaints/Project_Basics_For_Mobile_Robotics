# limit the angle to (-pi, pi)
import numpy as np
def convert_angle(angle):
    angle = angle % (2*np.pi)
    if angle >= np.pi:
        angle -= 2*np.pi
    return angle


class NoThymioError(Exception):
    pass
