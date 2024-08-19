import numpy as np

class PIDController:

    def __init__(self, kp, ki, kd, dt, alpha, integral_min, integral_max, output_min, output_max):

        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.integral = 0
        self.alpha = alpha
        self.integral_max = integral_max
        self.integral_min = integral_min
        self.output_max = output_max
        self.output_min = output_min
        self.derror =0.0
    def calculate_pid(self, errors):
        # Calculate proportional error
        proportional = self.kp * errors[-1]
        # Calculate integration error
        integral = self.integral + self.ki * (errors[-1]) * self.dt
        
        # Integral wineup
        if (integral > self.integral_max):
            integral = self.integral_max
        elif (integral < self.integral_min):
            integral = self.integral_min
        else:
            integral = integral
        self.integral = integral
        # Calculate derivative error
        error_est = errors[-1] - errors[-2]
        self.derror = self.derror*(1-self.alpha) + (self.alpha*error_est)
        # derivative = self.kd * (errors[-1] - errors[-2])/self.dt
        derivative = self.kd*(self.derror/self.dt)
        output = proportional + integral + derivative
        #Output Satuartion
        if (output > self.output_max):
            output = self.output_max
        elif (output < self.output_min):
            output = self.output_min
        else:
            output = output

        return output
    
if __name__ ==  "__main__":
    PID = PIDController()