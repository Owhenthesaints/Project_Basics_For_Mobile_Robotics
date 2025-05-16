import numpy as np
import time
import dependencies.constants_robot as cst


class ExtendedKalmanFilter:
    def __init__(self, position=np.array([0, 0, 0])):

        
        # R
        self.__measurement_covariance = np.diag([cst.RPX, cst.RPY, cst.RAN, cst.RLW, cst.RRW])
        self.__measurement_covariance_encoder = np.diag([cst.RLW, cst.RRW])
        # Q
        self.__process_noise_covariance = np.diag([cst.QPX, cst.QPY, cst.QAN, cst.QLW, cst.QRW])
        # f
        self.__state_transition_matrix = None
        # F
        self.__jacobian_state_matrix = None
        # h
        self.__measurement_vision = np.diag([1, 1, 1, 1, 1])
        self.__measurement_encoder = np.array([[0, 0, 0, 1, 0],
                                             [0, 0, 0, 0, 1]])
        # H
        self.__jacobian_measurement_vision = self.__measurement_vision
        self.__jacobian_measurement_encoder = self.__measurement_encoder
        #
        self.__last_t = time.time()
        self._dT = None
        self.init_state_vector(position)

    def init_state_vector(self, position, wheelspeed = None):
        if wheelspeed is not None:
            self._state = np.array([position[0], position[1], position[2], wheelspeed[0], wheelspeed[1]])
        else:
            self._state = np.array([position[0], position[1], position[2], 0, 0])
        self.__process_covariance = self.__measurement_covariance

    # get dT from last update
    def get_dt(self):
        timestamp = time.time()
        dt = timestamp - self.__last_t
        self.__last_t = timestamp
        self._dT = dt

    def set_state_transition_matrix(self):
        self.__state_transition_matrix = np.eye(len(self._state))
        self.__jacobian_state_matrix = np.eye(len(self._state))

        sin = np.sin(self._state[2])
        cos = np.cos(self._state[2])
        v_forward = (self._state[3] + self._state[4]) / 2

        self.__state_transition_matrix[0, 3] = -self._dT * cos / 2
        self.__state_transition_matrix[0, 4] = -self._dT * cos / 2
        self.__state_transition_matrix[1, 3] = self._dT * sin / 2
        self.__state_transition_matrix[1, 4] = self._dT * sin / 2
        # check - and + in practice
        self.__state_transition_matrix[2, 3] = -self._dT / cst.L1
        self.__state_transition_matrix[2, 4] = self._dT / cst.L1

        self.__jacobian_state_matrix[0, 2] = self._dT * v_forward * sin 
        self.__jacobian_state_matrix[1, 2] = self._dT * v_forward * cos

        self.__jacobian_state_matrix[0, 3] = -self._dT * cos / 2
        self.__jacobian_state_matrix[0, 4] = -self._dT * cos / 2
        self.__jacobian_state_matrix[1, 3] = self._dT * sin / 2
        self.__jacobian_state_matrix[1, 4] = self._dT * sin / 2
        # check - and + in practice
        self.__jacobian_state_matrix[2, 3] = -self._dT / cst.L1
        self.__jacobian_state_matrix[2, 4] = self._dT / cst.L1

        return self.__state_transition_matrix, self.__jacobian_state_matrix

    def get_state(self):
        return self._state[0:3], self.__process_covariance

    def predict(self):

        # dyanmic model
        f = self.__state_transition_matrix
        # Jacobian of the state transition function
        F = self.__jacobian_state_matrix

        # Prediction step
        self._state = f @ self._state
        # Prediction covariance
        self.__process_covariance = F @ self.__process_covariance @ F.T + self.__process_noise_covariance

    def update_vision(self, measurement):
        H = self.__jacobian_measurement_vision
        R = self.__measurement_covariance
        # Innovation or measurement residual
        
        measurement[3:5] = measurement[3:5] * cst.SPEED_FACTOR
        y = measurement - self.__measurement_vision @ self._state
        # Innovation (or residual) covariance
        S = H @ self.__process_covariance @ H.T + R
        # Compute the Kalman gain
        K = self.__process_covariance @ H.T @ np.linalg.inv(S)
        # Update the state estimate
        self._state = self._state + K @ y
        # Update the covariance matrix
        self.__process_covariance = (np.eye(len(self._state)) - K @ H) @ self.__process_covariance

        return self._state[0:3], self.__process_covariance
    
    def update_encoder(self, measurement):
        H = self.__jacobian_measurement_encoder
        R = self.__measurement_covariance_encoder
        # Innovation or measurement residual
        
        measurement[0:2] =  measurement[0:2]*cst.SPEED_FACTOR
        y = measurement - (self.__measurement_encoder @ self._state)
        # Innovation (or residual) covariance
        S = H @ self.__process_covariance @ H.T + R
        # Compute the Kalman gain
        K = self.__process_covariance @ H.T @ np.linalg.inv(S)
        # Update the state estimate
        self._state = self._state + K @ y
        # Update the covariance matrix
        self.__process_covariance = (np.eye(len(self._state)) - K @ H) @ self.__process_covariance

