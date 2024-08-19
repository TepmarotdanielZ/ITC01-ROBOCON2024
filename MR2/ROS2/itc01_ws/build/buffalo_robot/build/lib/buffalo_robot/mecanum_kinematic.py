import numpy as np

class Mecanum:
    def __init__ (self,r,lx,ly):
        self.r=r  # radius of Wheel
        self.lx=lx # distance from ceter robot to wheel on axis X
        self.ly=ly # distance from ceter robot to wheel on axis Y
    def inverse_kinematic(self,vx,vy,omega):
        #Motor1
        w1=(vx-vy-(self.lx+self.ly)*omega)/self.r
        #Motror2
        w2=(vx+vy+(self.lx+self.ly)*omega)/self.r
        #Motor3
        w3=(vx+vy-(self.lx+self.ly)*omega)/self.r
        #Motor4
        w4=(vx-vy+(self.lx+self.ly)*omega)/self.r
        return w1,w2,w3,w4
    def forward_kinematic(self,w1,w2,w3,w4):
        #Longitudinal_velocity
        Vx=(w1+w2+w3+w4)*(self.r/4)
        #Transversal Velocity
        Vy=(-w1+w2+w3-w4)*(self.r/4)
        #Angular Velocity
        Wz=(-w1+w2-w3+w4)*(self.r/(4*(self.lx+self.ly)))
        return Vx,Vy,Wz
    def discrete_state(self,x,y,yaw,w1,w2,w3,w4,dt):
        dx,dy,dyaw=self.forward_kinematic(w1,w2,w3,w4)
        x_next = x + dx * dt
        y_next = y + dy * dt
        yaw_next = yaw + dyaw * dt

        return x_next, y_next, yaw_next  
    
if __name__ ==  "__main__":
    Mecanum()