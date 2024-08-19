import numpy as np
import math
from buffalo_robot.omni_buffalo import Omni_model


class Extended_kalmanFilter:
    def __init__(self,Q,R,A):
        self.A = A
        self.H = np.eye(3)
        self.Q = Q
        self.R = R
        self.r = 0.05
        self.lx = 0.245/2
        self.ly = 0.245/2
        self.omni = Omni_model(self.r,self.lx,self.ly)
    def map(self,Input, min_input, max_input, min_output, max_output):
        value = ((Input - min_input)*(max_output-min_output)/(max_input - min_input) + min_output)
        return value
    def getB_Matrix(self,theta,type=None):
        rot = np.array([
            [ math.cos(theta),  -math.sin(theta),   0],
            [math.sin(theta),  math.cos(theta),   0],
            [0             ,              0,       1]
        ], dtype=np.float32)
        if (type == "mecanum"):
            J = (self.r/4)*np.array([[1, 1, 1, 1],
                                [1, -1, 1, -1],
                                [1/(self.lx+self.ly), -1/(self.lx+self.ly), -1/(self.lx+self.ly), 1/(self.lx+self.ly)]])
        if (type == "omni"):
            J = self.omni.forward_matrix("numpy")
        B = rot @ J
        return B
    def ekf_predicted(self,state,process_noise,P):
        # State estimate
        # B = self.getB_Matrix(state[2],model)
        
        # state_est = self.A @ state + dt*B @ input + process_noise
        # if (state_est[2,0] > 2*np.pi or state_est[2,0] < - 2*np.pi):
        #     state[2,0] = 0.0
        # state_est[2,0] = state_est[2,0]/2
        # if (model == "omni"):
        #     state_est = state + self.omni.forward_kinematic(input[0],input[1],input[2],input[3],state[2],"numpy")*dt + process_noise
        #     if (state_est[2] > 2*np.pi or state_est[2] < -2*np.pi):
        #         state_est[2] = 0.0
            
        #     if (state_est[2] >= np.pi):
        #         state_est[2] -= 2*np.pi  
        #     elif (state_est[2] < -np.pi): 
        #        state_est += 2*np.pi
        # Predicted estimate
        P_pred = self.A @ P @ self.A.T + self.Q

        return state,P_pred
    
    def ekf_update(self,state_pred,P_pred,measurement,measurement_noise):

        # Observation model
        Y = self.H @ state_pred + measurement_noise

        #Calculate error from sensor
        Y_error = measurement - Y 
        
        # Inovation Covariance
        S = self.H @ P_pred @ self.H.T + self.R

        # Kalman Gain
        K = P_pred @ self.H.T @ (np.linalg.pinv(S))

        # State Update
        State_up = state_pred + K @ Y_error

        # Predicted update
        P_update = (np.identity(3) - (K @ self.H)) @ P_pred

        return State_up, P_update


if __name__ == "__main__":
    EKF = Extended_kalmanFilter()



        
        
        