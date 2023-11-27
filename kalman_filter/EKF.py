import numpy as np
import dependencies.constants_robot as cst
import time


class ExtendedKalmanFilter:
    def __init__(self, position):

        self.__I = np.eye(5)
        # x = [posx, posy, angle, left_wheel_speed, right_wheel_speed]
        self.x = np.zeros(5)
        self.__F = np.eye(5)
        self.__f = np.eye(5)
        # store previous time stamp to compute dt
        self.timestamp = 0
        self.__H = np.eye(5, dtype=int)
        # if without camera vision, use only [left_wheel_speed, right_wheel_speed] to propagate
        self.__H_nocamera = np.array([[0, 0, 0, 1, 0],
                                    [0, 0, 0, 0, 1]], dtype=int).T

        # noise matrices
        self.__R = np.diag([cst.RPX, cst.RPY, cst.RAN, cst.RLW, cst.RRW])
        self.__R_nocamera = np.diag([cst.RLW, cst.RRW])
        self.__Q = np.diag([cst.QPX, cst.QPY, cst.QAN, cst.QLW, cst.QRW])

        # initialization of matrix P in EKF
        # P = var(x(t0)), firstly assumed to be the same as Q
        self.__P = self.__Q
        self.update_t(time.time())
        self.init_state_vector(position, np.array([0, 0]))




    def init_state_vector(self, pos, speed):
        # pos = [posx, posy, angle]
        self.x = np.concatenate((pos, speed), dtype=np.float64)

    def last_t(self):
        return self.timestamp

    def update_t(self, timestamp):
        self.timestamp = timestamp
   
    def recompute_F(self, dt):
        '''
        x(k) = f(x(k-1), u(t)) + q, q~N(0, Q)
        y(k) = h(x(k))       + r, r~N(0, R)
        F = df/dx
        '''
        angle = self.x[2]
        dcosA = dt*np.cos(angle)
        dsinA = dt*np.sin(angle)
        mSpeed = (self.x[3]+self.x[4])/2

        # f(x) = x @ self.f
        self.__f = np.array([
                        [      1,       0,     0, 0, 0],
                        [      0,       1,     0, 0, 0],
                        [      0,       0,     1, 0, 0],
                        [dcosA/2, dsinA/2,  dt/cst.L, 1, 0],
                        [dcosA/2, dsinA/2, -dt/cst.L, 0, 1]])
        # may have to change the sign of both dt/cst.L
        
        # Transspose of jacobian matrix of f
        self.__F = np.array([
                        [            1,            0,     0, 0, 0],
                        [            0,            1,     0, 0, 0],
                        [-mSpeed*dsinA, mSpeed*dcosA,     1, 0, 0],
                        [      dcosA/2,      dsinA/2,  dt/cst.L, 1, 0],
                        [      dcosA/2,      dsinA/2, -dt/cst.L, 0, 1]])

    def current_estimate(self):
            return self.x

    def predict(self):
        self.x = self.x @ self.__f
        self.__P = (self.__F.T @ self.__P @ self.__F) + self.__Q

    def update(self, has_vision, pos_measure, speed):
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

        corr_wspeed = np.array(speed)*cst.CORR_FACTOR

        if has_vision:
            y = np.concatenate((pos_measure, corr_wspeed)) - (self.x @ self.__H)
            S = (self.__H.T @ self.__P @ self.__H) + self.__R
            K = np.linalg.inv(S) @ self.__H.T @ self.__P
            self.x += y @ K
            self.__P = self.__P @ (self.__I - self.__H @ K)
            
        else:
            y = corr_wspeed - (self.x @ self.__H_nocamera)
            # pre compute for the kalman gain K
            S = (self.__H_nocamera.T @ self.__P @ self.__H_nocamera) + self.__R_nocamera
            K = np.linalg.inv(S) @ self.__H_nocamera.T @ self.__P
            # now we update our prediction using the error and kalman gain.
            self.x += y @ K
            self.__P = self.__P @ (self.__I - self.__H_nocamera @ K)