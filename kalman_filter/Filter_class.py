import numpy as np
import time
import dependencies.constants_robot as cst


class ExtendedKalmanFilter:
    def __init__(self, position=np.array([0, 0, 0])):

        
        # R
        self.measurement_covariance = np.diag([cst.RPX, cst.RPY, cst.RAN, cst.RLW, cst.RRW])
        self.measurement_covariance_encoder = np.diag([cst.RLW, cst.RRW])
        # Q
        self.process_noise_covariance = np.diag([cst.QPX, cst.QPY, cst.QAN, cst.QLW, cst.QRW])
        # f
        self.state_transition_matrix = None
        # F
        self.jacobian_state_matrix = None
        # h
        self.measurement_vision = np.diag([1, 1, 1, 1, 1])
        self.measurement_encoder = np.array([[0, 0, 0, 1, 0],
                                             [0, 0, 0, 0, 1]])
        # H
        self.jacobian_measurement_vision = self.measurement_vision
        self.jacobian_measurement_encoder = self.measurement_encoder
        #
        self.last_t = time.time()
        self.dT = None
        self.init_state_vector(position)

    def init_state_vector(self, position, wheelspeed = None):
        if wheelspeed is not None:
            self.state = np.array([position[0], position[1], position[2], wheelspeed[0], wheelspeed[1]])
        else:
            self.state = np.array([position[0], position[1], position[2], 0, 0])
        self.process_covariance = self.measurement_covariance

    # get dT from last update
    def get_dt(self):
        timestamp = time.time()
        dt = timestamp - self.last_t
        self.last_t = timestamp
        self.dT = dt

    def set_state_transition_matrix(self):
        self.state_transition_matrix = np.eye(len(self.state))
        self.jacobian_state_matrix = np.eye(len(self.state))

        sin = np.sin(self.state[2])
        cos = np.cos(self.state[2])
        v_forward = (self.state[3] + self.state[4]) / 2

        self.state_transition_matrix[0, 3] = self.dT * cos / 2
        self.state_transition_matrix[0, 4] = self.dT * cos / 2
        self.state_transition_matrix[1, 3] = self.dT * sin / 2
        self.state_transition_matrix[1, 4] = self.dT * sin / 2
        # check - and + in practice
        self.state_transition_matrix[2, 3] = self.dT / cst.L
        self.state_transition_matrix[2, 4] =  -self.dT / cst.L

        self.jacobian_state_matrix[0, 2] = - self.dT * v_forward * sin 
        self.jacobian_state_matrix[0, 3] = self.dT * cos / 2
        self.jacobian_state_matrix[0, 4] = self.dT * cos / 2
        self.jacobian_state_matrix[1, 2] = self.dT * v_forward * cos
        self.jacobian_state_matrix[1, 3] = self.dT * sin / 2
        self.jacobian_state_matrix[1, 4] = self.dT * sin / 2
        # check - and + in practice
        self.jacobian_state_matrix[2, 3] = self.dT / cst.L
        self.jacobian_state_matrix[2, 4] = -self.dT / cst.L

        return self.state_transition_matrix, self.jacobian_state_matrix

    def get_state(self):
        return self.state, self.process_covariance

    def predict(self):

        # dyanmic model
        f = self.state_transition_matrix
        # Jacobian of the state transition function
        F = self.jacobian_state_matrix

        # Prediction step
        self.state = f @ self.state
        # Prediction covariance
        self.process_covariance = F @ self.process_covariance @ F.T + self.process_noise_covariance

    def update_vision(self, measurement):
        H = self.jacobian_measurement_vision
        R = self.measurement_covariance
        # Innovation or measurement residual
        speed_factor = 1/3
        y = measurement - self.measurement_vision @ self.state
        # Innovation (or residual) covariance
        S = H @ self.process_covariance @ H.T + R
        # Compute the Kalman gain
        K = self.process_covariance @ H.T @ np.linalg.inv(S)
        # Update the state estimate
        self.state = self.state + K @ y
        # Update the covariance matrix
        self.process_covariance = (np.eye(len(self.state)) - K @ H) @ self.process_covariance

        return self.state, self.process_covariance
    
    def update_encoder(self, measurement):
        H = self.jacobian_measurement_encoder
        R = self.measurement_covariance_encoder
        # Innovation or measurement residual
        speed_factor = 1/3
        y = measurement - (self.measurement_encoder @ self.state)
        # Innovation (or residual) covariance
        S = H @ self.process_covariance @ H.T + R
        # Compute the Kalman gain
        K = self.process_covariance @ H.T @ np.linalg.inv(S)
        # Update the state estimate
        self.state = self.state + K @ y
        # Update the covariance matrix
        self.process_covariance = (np.eye(len(self.state)) - K @ H) @ self.process_covariance

        return self.state, self.process_covariance