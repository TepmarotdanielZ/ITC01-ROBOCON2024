import numpy as np

class OmniModel:
    def __init__(self,R,Lx,Ly):
        self.R = R
        self.L = Lx + Ly
        self.a1 = np.pi/4
        self.a2 = 3*np.pi/4
        self.a3 = 5*np.pi/4
        self.a4 = 7*np.pi/4

    def Rotation_matrix(self,theta):
        R = np.array([[np.cos(theta),np.sin(theta),0.0],
                     [-np.sin(theta), np.cos(theta), 0.0],
                     [0.0, 0.0, 1.0]])
        return R
    
    def Forward_kinematic(self,W1,W2,W3,W4,angle,theta=0.0):
        Rot = self.Rotation_matrix(angle)
        Vx = (self.R/2)*(W1*np.sin(theta+self.a1) + W2*np.sin(theta+self.a2) - W3*np.sin(theta+self.a3) - W4*np.sin(theta+self.a4))
        Vy = (self.R/2)*(-W1*np.cos(theta+self.a1) - W2*np.cos(theta+self.a2) + W3*np.cos(theta+self.a3) + W4*np.cos(theta+self.a4))
        Omega = (self.R/2)*(-(W1/(2*self.L)) + (W2/(2*self.L)) +  (W3/(2*self.L)) - (W4/(2*self.L)))
        V = (np.array([[Vx],[Vy],[Omega]]))
        return V
    
    def Inverse_kinematic(self,Vx,Vy,Omega,theta=0.0):
        W1 = (1/self.R)*(Vx*np.sin(theta+self.a1) - Vy*np.cos(theta+self.a1) - Omega*(self.L)) 
        W2 = (1/self.R)*(Vx*np.sin(theta+self.a2) - Vy*np.cos(theta+self.a2) + Omega*(self.L))
        W3 = (1/self.R)*(-Vx*np.sin(theta+self.a3) + Vy*np.cos(theta+self.a3) + Omega*(self.L))
        W4 = (1/self.R)*(-Vx*np.sin(theta+self.a4) + Vy*np.cos(theta+self.a4) - Omega*(self.L)) 
        W = np.array([[W1],[W2],[W3],[W4]]) 
        return W
    
    def discrete_state(self,x,y,yaw,w1,w2,w3,w4,dt):
        V=self.Forward_kinematic(w1,w2,w3,w4,yaw)
        x_next = x+ V[0,0]*dt
        y_next = y+ V[1,0]*dt
        yaw_next = yaw + V[2,0]*dt
        return x_next,y_next,yaw_next    
    
if __name__ == "__main__":
    OmniModel()