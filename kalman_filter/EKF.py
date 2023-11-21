import numpy as np

l = 90 # in mm, dist between wheel centers

# speed variance obtanied 12.31 mm^2/s^2
SPEED_VAR = 12.31
# need tuning in practice
POS_VAR = 2**2
ANGLE_VAR = (np.pi/10)**2
CORR_FACTOR = 2


RPX = POS_VAR
RPY = POS_VAR
RAN = ANGLE_VAR
RLW = SPEED_VAR
RRW = SPEED_VAR


# assume less certain about the model used
QPX = POS_VAR*4
QPY = POS_VAR*4
QAN = ANGLE_VAR*4
QLW = SPEED_VAR*4
QRW = SPEED_VAR*4

class ExtendedKalmanFilter:
    def __init__(self):

        # do this once do we don't keep redoing in update step
        self.__I = np.eye(5)

        self.__x = np.zeros(5)
        self.__F = np.eye(5)
        self.__f = np.eye(5)
        # store previous time stamp to compute dt
        self.__timestamp = 0
        # estimation state
        # pos_x, pos_y, angle, vel_leftwheel, vel_rightwheel
        self.__H = np.eye(5, dtype=int)
        self.__H_nocamera = np.array([[0, 0, 0, 1, 0],
                                    [0, 0, 0, 0, 1]], dtype=int).T

        # noise matrices
        self.__R = np.diag([RPX, RPY, RAN, RLW, RRW])
        self.__R_nocamera = np.diag([RLW, RRW])
        self.__Q = np.diag([QPX, QPY, QAN, QLW, QRW])

        # initialization of matrix P in EKF
        # P = var(x(t0))
        # firstly assumed to be the same as Q
        self.__P = self.__Q
        #self.P = np.eye(5, dtype=int)



    def init_state_vector(self, pos, angle, speed):
        self.__x = np.concatenate((pos, [angle], speed), dtype=np.float64)

    def last_t(self):
        return self.__timestamp

    def update_t(self, timestamp):
        self.__timestamp = timestamp
   
    def recompute_F(self, dt):
        '''
        x(k) = f(x(k-1), u(t)) + q, q~N(0, Q)
        y(k) = h(x(k))       + r, r~N(0, R)
        F = df/dx
        '''
        angle = self.__x[2]
        dcosA = dt*np.cos(angle)
        dsinA = dt*np.sin(angle)
        mSpeed = (self.__x[3]+self.__x[4])/2

        # f(x) = x @ self.f
        self.__f = np.array([
                        [      1,       0,     0, 0, 0],
                        [      0,       1,     0, 0, 0],
                        [      0,       0,     1, 0, 0],
                        [dcosA/2, dsinA/2,  dt/l, 1, 0],
                        [dcosA/2, dsinA/2, -dt/l, 0, 1]])
        
        # Transspose of jacobian matrix of f
        self.__F = np.array([
                        [            1,            0,     0, 0, 0],
                        [            0,            1,     0, 0, 0],
                        [-mSpeed*dsinA, mSpeed*dcosA,     1, 0, 0],
                        [      dcosA/2,      dsinA/2,  dt/l, 1, 0],
                        [      dcosA/2,      dsinA/2, -dt/l, 0, 1]])

    def current_estimate(self):
            return self.__x

    def predict(self):
        self.__x = self.__x @ self.__f
        self.__P = (self.__F.T @ self.__P @ self.__F) + self.__Q

    def update(self, has_vision, pos_sensor, angle_sensor, speed):
        '''
        x(k) = f(x(k-1), u(t)) + q, q~N(0, Q)
        y(k) = h(x(k))       + r, r~N(0, R)
        F = df/dx
        H = dh/dx
        P = FPF'+Q
        S = HPH'+R
        K = PH'(S^-1)
        x(k) = x(k) + Ky
        P = (xI - KH)P
        '''

        corr_wspeed = np.array(speed)*CORR_FACTOR

        if has_vision:
            y = np.concatenate((pos_sensor, [angle_sensor], corr_wspeed)) - (self.__x @ self.__H)
            S = (self.__H.T @ self.__P @ self.__H) + self.__R
            K = np.linalg.inv(S) @ self.__H.T @ self.__P
            self.__x += y @ K
            self.__P = self.__P @ (self.__I - self.__H @ K)
            
        else:
            y = corr_wspeed - (self.__x @ self.__H_nocamera)
            # pre compute for the kalman gain K
            S = (self.__H_nocamera.T @ self.__P @ self.__H_nocamera) + self.__R_nocamera
            K = np.linalg.inv(S) @ self.__H_nocamera.T @ self.__P
            # now we update our prediction using the error and kalman gain.
            self.__x += y @ K
            self.__P = self.__P @ (self.__I - self.__H_nocamera @ K)