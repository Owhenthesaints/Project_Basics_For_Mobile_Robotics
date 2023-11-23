# limit the angle to (-pi, pi)
def convert_angle(angle):
    angle = angle % (2*np.pi)
    if angle >= np.pi:
        angle -= 2*np.pi
    return angle
