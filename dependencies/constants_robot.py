# everything is now in radians from pi to -pi with righthand rule starting with the robot looking down x
import numpy as np
# the parameters you should look at (under)

####################################  ThymioRobot class  ##########
DEFAULT_KAPPA_ALPHA = 250
DEFAULT_KAPPA_BETA = 250
DEFAULT_KAPPA_RHO = 5
DEFAULT_THRESHOLD = 1
LOCAL_NAV_SWITCH = 1000
LOCAL_NAV_FORWARD_SPEED = 70
LOCAL_NAV_FACTOR = 0.01

####################################  Kalman filter class  ##########
# speed variance obtanied 12.31 mm^2/s^2
SPEED_VAR = 12.31
POSITION_VAR = 2**2
ANGLE_VAR = (np.pi/10)**2
CORR_FACTOR = 2                  # also for motion control : correlation factor of thymio real speed and wheel pwm

# uncertainty about measurement
RPX = POSITION_VAR
RPY = POSITION_VAR
RAN = ANGLE_VAR
RLW = SPEED_VAR
RRW = SPEED_VAR

# uncertainty about dynamic model
# assume less certain about the model used
QPX = POSITION_VAR*4
QPY = POSITION_VAR*4
QAN = ANGLE_VAR*4
QLW = SPEED_VAR*4
QRW = SPEED_VAR*4

# thresholds for deciding kidnapping
DIST_TRESHOLD = 100
ANGLE_TRESHOLD = np.pi/4

####################################  local navigation  ##########
#Threshold to switch states
OBSTTHRH = 2000
OBSTTHRL = 1000

# obstacle avoidance: ANN weights
w = np.array(
[[50, 30, -20, -30, -50, 30, -10, 8, 0],
[-50, -30, -20, 30, 50, -10, 30, 0, 8]]) 

# Scale factors that divide sensor inputs and memory inputs
SENSOR_SCALE = 800
MEMORY_SCALE = 20







# no need to touch these(under)
DELTA_ROBOT = 1.35
REFL_ROBOT = 1.35
L = 90                           # in mm, dist between wheel centers

# limit the angle to (-pi, pi) 
def convert_angle(angle):
    angle = angle % (2*np.pi)
    if angle >= np.pi:
        angle -= 2*np.pi
    return angle
