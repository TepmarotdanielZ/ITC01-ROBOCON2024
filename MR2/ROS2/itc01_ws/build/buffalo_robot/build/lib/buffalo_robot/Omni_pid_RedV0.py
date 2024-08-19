import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Float32MultiArray,Float32,Int8,String,UInt16,UInt8MultiArray,Int8MultiArray, UInt8
from sensor_msgs.msg import Imu
import numpy as np
from buffalo_robot.pid_controller import PIDController
from buffalo_robot.omni_buffalo import Omni_model
import matplotlib.pyplot as plt
import math
import time


class Buffalo_PID(Node):
    def __init__(self):
        super().__init__('Bf_PID')
        
        ###**** SUBSCRIPTION *****######
        self.mode_sub = self.create_subscription(String, '/mode', self.subscrib_mode, 10)
        self.manual_sub = self.create_subscription(Twist,'/cmd_vel', self.subscrib_manual, 10)
        self.feedback_sub = self.create_subscription(Float32MultiArray, '/external', self.External_feedback, 10)
    
        self.sub_typeball = self.create_subscription(String, '/ball_class', self.balltype_callback, 10)
        self.ball_aready_sub = self.create_subscription(UInt8MultiArray, '/ball_already',self.ball_already_callback, 10)
        self.goal_sub = self.create_subscription(Float32MultiArray, '/goal_end', self.goal_callback, 10)
        self.state_sub = self.create_subscription(UInt16 , '/state', self.state_callback , 10)
        # self.sub_imu = self.create_subscription(Vector3, "/imu/data", self.Imu_callback, 20)
        ###**** PUBLISHER ****#####
        self.input_pub = self.create_publisher(Float32MultiArray, '/control', 10)
        ###*** TIMER ****####
        timer = 0.01
        self.output_timer = self.create_timer(timer,self.Calculate_position)
        
        ###*** VARIABLE PARAMETER ***###
        ### Parameter of Robot
        self.R = 0.06
        self.Lx = 0.45/2
        self.Ly = 0.45/2 
        self.omni=Omni_model(self.R,self.Lx,self.Ly)
        self.sampling_time = 0.01
        
        ## Variable PID Auto
        self.dt = 0.01
        self.Kp = np.array([2.5, 2.2, 1.2])
        self.Ki = np.array([0.0001, 0.0001, 0.0001])
        self.Kd = np.array([0.008, 0.008, 0.008])
        self.integral_min = -1.5
        self.integral_max = 1.5
        self.output_min = -1.5
        self.output_max = 1.5
        self.integral_min_Angular = -0.7
        self.integral_max_Angular = 0.7
        self.output_min_Angular = -0.7
        self.output_max_Angular = 0.7
        self.alpha = 0.7
        self.pid_X= PIDController(self.Kp[0], self.Ki[0], self.Kd[0], self.dt, self.alpha, self.integral_min, self.integral_max, self.output_min, self.output_max)
        self.pid_Y= PIDController(self.Kp[1], self.Ki[1], self.Kd[1], self.dt, self.alpha, -1.4, 1.4, -1.4, 1.4)
        self.pid_Yaw= PIDController(self.Kp[2], self.Ki[2], self.Kd[2], self.dt, self.alpha, self.integral_min_Angular, self.integral_max_Angular, self.output_min_Angular, self.output_max_Angular)
        self.output_vx = 0.0
        self.output_vy = 0.0
        self.output_omega = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.omega = 0.0
        # Manual
        self.Vx = 0.0
        self.Vy = 0.0
        self.Omega = 0.0
        
        ## GOAL POINT AUTO
        self.prev_time = time.time()
        self.prev_timeball = time.time()
        # self.prev_timeBall = time.time()
        
        self.sampling_time = 0.01
        self.mode_status = "auto"
        self.goal = None
        self.X_start = 0.0
        self.Y_start = 0.0
        self.Yaw_start = 0.0
        
        self.state_end = [0.0, 0.0, 0.0]
        self.state_start = [0.0 , 0.0, 0.0]
        self.current_x = -0.00
        self.current_y = 0.00
        self.current_yaw = 0.0
        self.errorX = [self.state_end[0] - self.current_x]
        self.errorY = [self.state_end[1] - self.current_y]
        self.errorYaw = [self.state_end[2] - self.current_yaw]
        self.errorx = 0.0
        self.errory = 0.0
        self.erroryaw = 0.0
        self.state = 0
        self.goal_limitball = [0.0, 0.0]
        
        ## Variable sensor feedback
        self.Vx_ex = 0.0
        self.Vy_ex = 0.0
        self.yaw_imu = 0.0
        self.V_m = np.array([[0.0],[0.0],[0.0]])
        self.W1 = 0.0
        self.W2 = 0.0
        self.W3 = 0.0
        self.W4 = 0.0
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.distance = 0.0
        self.silo_goal = 0
        self.button = 0
        
        self.typeball = None
        
        
    def External_feedback(self, current):
        self.current_x = current.data[0]
        self.current_y = current.data[1]
        self.current_yaw = current.data[2]
        
    def Encoder_feedback(self, W):
        self.W1 = W.data[0]
        self.W2 = W.data[1]
        self.W3 = W.data[2]
        self.W4 = W.data[3]
    
    def ball_already_callback(self, ball):
        self.ball_aready = ball.data[0]
        self.button = ball.data[1]
        self.check_three_ball = ball.data[2]
        
        # self.get_logger().info('Ball_inside :"%d"' %self.ball_aready)
        if (self.ball_aready == 1 and self.state == 5):
            self.state = 6
            self.mode_status = "auto"
        if (self.button == 1):
            self.goal = "success"
            self.goal_limitball = [7.5, 13.5] # [min, max]
        elif (self.button == 2):
            self.goal = "retry"
            self.goal_limitball = [1.5, 6.5] # [min, max]
        
    def subscrib_manual(self, V):
        self.Vx = V.linear.x
        self.Vy = V.linear.y
        self.Omega = V.angular.z
        
    def subscrib_mode(self, mode):
        self.mode_status = mode.data
    
    def balltype_callback(self, type):
        self.typeball = type.data
    def state_callback(self, State):
        self.state = State.data
    def goal_callback(self, goal):
        self.state_end[0] = goal.data[0]
        self.state_end[1] = goal.data[1]
        self.state_end[2] = goal.data[2]
    def Calculate_position(self):
        self.errorx = self.state_end[0] - self.current_x
        self.errory = self.state_end[1] - self.current_y
        self.erroryaw = self.state_end[2] - self.current_yaw
        # When Catch Ball
        # if (self.current_x >= self.goal_limitball[0] and self.current_x < self.goal_limitball[1] and self.current_y >= -4.8 and self.current_y < 0.5 and (self.state >= 5 and self.state <= 7)):
        if ((self.state == 5)):
            self.pid_X= PIDController(0.65, self.Ki[0], self.Kd[0], self.dt, self.alpha, -0.9, 0.9, -0.9, 0.9)
            self.pid_Y= PIDController(0.65, self.Ki[1], self.Kd[1], self.dt, self.alpha, -0.9, 0.9, -0.9, 0.9)
            self.pid_Yaw= PIDController(1.8, self.Ki[2], self.Kd[2], self.dt, self.alpha, -0.9, 0.9, -0.9, 0.9)
        # When catch already and to take ball
        elif(self.current_x >= self.goal_limitball[0] and self.current_x <= self.goal_limitball[1] and self.current_y >= -10.0 and self.current_y < 0.5 and ((self.state >= 6 and self.state<=9))):
            self.pid_X= PIDController(1.5, self.Ki[0], self.Kd[0], self.dt, self.alpha, -1.2, 1.2, -1.2, 1.2)
            self.pid_Y= PIDController(1.5, self.Ki[1], self.Kd[1], self.dt, self.alpha, -1.2, 1.2, -1.2, 1.2)
            self.pid_Yaw= PIDController(1.8, self.Ki[2], self.Kd[2], self.dt, self.alpha, -1.0, 1.0, -1.0, 1.0)
        # When Find silo
        elif(self.current_x >= self.goal_limitball[0] and self.current_x <= self.goal_limitball[1] and self.current_y >= -10.0 and self.current_y < 0.5 and (self.state >= 10 and self.state<=15)):
            self.pid_X= PIDController(2.5, self.Ki[0], self.Kd[0], self.dt, self.alpha, -1.0, 1.0, -1.0, 1.0)
            self.pid_Y= PIDController(2.5, self.Ki[1], self.Kd[1], self.dt, self.alpha, -1.0, 1.0, -1.0, 1.0)
            self.pid_Yaw= PIDController(1.0, self.Ki[2], self.Kd[2], self.dt, self.alpha, -0.7, 0.7, -0.7, 0.7)
        self.errorX.append(self.state_end[0] - self.current_x)
        self.errorY.append(self.state_end[1] - self.current_y)
        self.errorYaw.append(self.state_end[2] - self.current_yaw)

        self.output_vx = self.pid_X.calculate_pid(self.errorX)
        self.output_vy = self.pid_Y.calculate_pid(self.errorY)
        self.output_omega = self.pid_Yaw.calculate_pid(self.errorYaw)

        self.vx = self.output_vx*np.cos(self.current_yaw)  + self.output_vy*np.sin(self.current_yaw)
        self.vy = -self.output_vx*np.sin(self.current_yaw) + self.output_vy*np.cos(self.current_yaw)
        self.omega = self.output_omega
        
        if (self.current_y >=-3.0 and self.current_y <=0.1 and abs(self.erroryaw) >= 0.25 and self.state == 5 and self.typeball == "red ball"):
            self.vx = 0.0
            self.vy = 0.0
            self.omega = self.output_omega
            
        # TRANSMITION Velocity When "MANUAL"
        if (self.mode_status == "manual"):
            V = Float32MultiArray()
            W = self.omni.inverse_kinematic(self.Vx,self.Vy,self.Omega,"numpy")
            V.data = [W[0],W[1],W[2],W[3]]  
            self.input_pub.publish(V)
            self.get_logger().info('Velocity_manual : "[%f, %f, %f, %f]"' % (W[0], W[1], W[2], W[3]))
        
        # TRANSMITION Velocity When "AUTO"
        elif (self.mode_status == "auto"):
            V = Float32MultiArray()
            W = self.omni.inverse_kinematic(self.vx,self.vy,self.omega,"numpy")
            if (W[0] < 0.5 and W[0] > - 0.5):
                W[0] = 0.0
            if (W[1] < 0.5 and W[1] > - 0.5):
                W[1] = 0.0
            if (W[2] < 0.5 and W[2] > - 0.5):
                W[2] = 0.0
            if (W[3] < 0.5 and W[3] > - 0.5):
                W[3] = 0.0
            V.data = [W[0],W[1],W[2],W[3]] 
            self.get_logger().info('Velocity_auto : "[%f, %f, %f, %f]"' % (W[0], W[1], W[2], W[3]))
            self.input_pub.publish(V)

def main(args=None):
    rclpy.init(args=args)
    pid_bf = Buffalo_PID()
    rclpy.spin(pid_bf)
    pid_bf.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()