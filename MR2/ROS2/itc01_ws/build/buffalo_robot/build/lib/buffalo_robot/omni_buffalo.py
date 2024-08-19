import numpy as np
import casadi as ca

class Omni_model():
    def __init__(self,R,Lx,Ly):
        self.R = R
        self.L = Lx + Ly

    def forward_matrix(self, type=None):
        if type == "numpy":
            J = (self.R/2)*np.array([[-0.7, -0.7, 0.7, 0.7],
                                    [ 0.7, -0.7,-0.7, 0.7],
                                    [1/(2*self.L), 1/(2*self.L), 1/(2*self.L), 1/(2*self.L)]],dtype=np.float64)
        elif type == "sym":
            J = (self.R/2)*ca.vertcat(
                ca.horzcat(-0.7, -0.7, 0.7, 0.7),
                ca.horzcat(0.7, -0.7, -0.7, 0.7),
                ca.horzcat(1/(2*self.L), 1/(2*self.L), 1/(2*self.L), 1/(2*self.L))
            )
        return J
    
    def inverse_matrix(self, type=None):
        
        if type == "numpy":
            J_inv = (1/self.R)*np.array([[-0.7, 0.7, self.L],
                                         [-0.7,-0.7, self.L],
                                         [ 0.7,-0.7, self.L],
                                         [ 0.7, 0.7, self.L]],dtype=np.float64)
            
        elif type == "sym":
            J_inv = (1/self.R)*ca.vertcat(
                ca.horzcat(-0.7, 0.7, self.L),
                ca.horzcat(-0.7,-0.7, self.L),
                ca.horzcat( 0.7,-0.7, self.L),
                ca.horzcat( 0.7, 0.7, self.L)
            )

        return J_inv
    
    def rotation_matrix(self,theta, type=None):

        if type == "numpy":
            rot = np.array([[ np.cos(theta), np.sin(theta), 0],
                            [-np.sin(theta), np.cos(theta), 0],
                            [0, 0, 1]], dtype=np.float64)
            
        elif type == "sym":
            rot = ca.vertcat(
                ca.horzcat( ca.cos(theta), ca.sin(theta), 0),
                ca.horzcat(-ca.sin(theta), ca.cos(theta), 0),
                ca.horzcat(0, 0, 1)
            )
        return rot
    
    def forward_kinematic(self, W1, W2, W3, W4, theta, type=None):
        rot = self.rotation_matrix(theta, type)
        if type == "numpy": 
            V = rot.T @ self.forward_matrix(type) @ np.array([W1,W2,W3,W4])

        elif type == "sym":
            V = rot.T @ self.forward_matrix(type) @ ca.vertcat(W1,W2,W3,W4)

        return V
    
    def inverse_kinematic(self, Vx, Vy, Omega, type=None):
        
        if type == "numpy":
            W = self.inverse_matrix(type) @ np.array([Vx,Vy,Omega])

        elif type == "sym":
            W = self.inverse_matrix(type) @ ca.vertcat(Vx,Vy,Omega)

        return W

if __name__ ==  "__main__":
    buffalo = Omni_model()
