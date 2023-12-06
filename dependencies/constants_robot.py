# everything is now in radians from pi to -pi with righthand rule starting with the robot looking down x
import numpy as np
# the parameters you should look at (under)

####################################  ThymioRobot class  ##########
DEFAULT_KAPPA_ALPHA = 300 *6
DEFAULT_KAPPA_BETA = - 70 *0.2
DEFAULT_KAPPA_RHO = 1
DEFAULT_THRESHOLD = 40
LOCAL_NAV_SWITCH = 1000
LOCAL_NAV_FORWARD_SPEED = 70
LOCAL_NAV_FACTOR = 0.01
MAX_GRID = [640, 480]

####################################  Kalman filter class  ##########
# speed variance obtanied 12.304 mm^2/s^2
SPEED_VAR = 12.30
POSITION_VAR = 4
ANGLE_VAR = 0.09
SPEED_CONVERT = 0.43
PIXEL_SPEED_FACTOR = 0.41
SPEED_FACTOR = SPEED_CONVERT*PIXEL_SPEED_FACTOR                 
# SPEED_FACTOR = 1/3  
# uncertainty about measurement
RPX = POSITION_VAR
RPY = POSITION_VAR
RAN = ANGLE_VAR
RLW = SPEED_VAR/2
RRW = SPEED_VAR/2

# uncertainty about dynamic model
# assume less certain about the model used
QPX = POSITION_VAR*5
QPY = POSITION_VAR*5
QAN = ANGLE_VAR*5
QLW = SPEED_VAR*5
QRW = SPEED_VAR*5

# thresholds for deciding kidnapping
DIST_TRESHOLD = 100
ANGLE_TRESHOLD = np.pi/4

####################################  local navigation  ##########
LOCAL_NAV_SWITCH = 1000
# Scale factors that divide sensor inputs and memory inputs
# SENSOR_SCALE = 800
# MEMORY_SCALE = 20







# no need to touch these(under)
DELTA_ROBOT = 1.35
REFL_ROBOT = 1.35
L = 90
L1 = L*0.5

