import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Float32MultiArray,Float32,Int8,String
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
        self.manual_sub = self.create_subscription(Twist,'/manual', self.subscrib_manual, 10)
        self.feedback_sub = self.create_subscription(Float32MultiArray, '/external', self.External_feedback, 10)
        self.encoder_sub = self.create_subscription(Float32MultiArray, '/encoder', self.Encoder_feedback, 10)
        self.laser_sub = self.create_subscription(Float32MultiArray, '/laser', self.laser_callback, 10)
        self.sub_imu = self.create_subscription(Vector3, "/imu/data", self.Imu_callback, 20)
        ###**** PUBLISHER ****#####
        self.input_pub = self.create_publisher(Float32MultiArray, '/control', 10)
        
        ###*** TIMER ****####
        timer = 0.01
        self.output_timer = self.create_timer(timer,self.Calculate_position)
        self.ball_timer = self.create_timer(timer,self.calculate_goal)
        
        ###*** VARIABLE PARAMETER ***###
        ### Parameter of Robot
        self.R = 0.06
        self.Lx = 0.45/2
        self.Ly = 0.45/2 
        self.omni=Omni_model(self.R,self.Lx,self.Ly)
        self.sampling_time = 0.01
        
        ## Variable PID Auto
        self.dt = 0.01
        self.Kp = np.array([2.0, 2.0, 1.0])
        self.Ki = np.array([0.0001, 0.0001, 0.0001])
        self.Kd = np.array([0.008, 0.008, 0.01])
        self.integral_min = -1.8
        self.integral_max = 1.8
        self.output_min = -1.8
        self.output_max = 1.8
        self.integral_min_Angular = -0.7
        self.integral_max_Angular = 0.7
        self.output_min_Angular = -0.7
        self.output_max_Angular = 0.7
        self.alpha = 0.7
        self.pid_X= PIDController(self.Kp[0], self.Ki[0], self.Kd[0], self.dt, self.alpha, self.integral_min, self.integral_max, self.output_min, self.output_max)
        self.pid_Y= PIDController(self.Kp[1], self.Ki[1], self.Kd[1], self.dt, self.alpha, -1.3, 1.3, -1.3, 1.3)
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
        
        self.sampling_time = 0.01
        self.mode_status = "auto"
        self.goal = "goal1"
        self.X_start = 0.0
        self.Y_start = 0.0
        self.Yaw_start = 0.0
        self.Goal = np.array([0.0, -0.0, 0.0])
        self.goal0 = np.array([6.1, 0.0, 0.0])
        self.goal1 = np.array([6.1, -3.7, 0.0])
        self.goal2 = np.array([9.65, -3.7, 0.0])
        self.goal3 = np.array([9.65, -1.0, 1.57])
        self.goal4 = np.array([9.65, -1.0, 1.57])
        self.goalball = np.array([0.0,0.0,0.0])
        self.silo1 = np.array([8.15, -4.38, 1.57])
        self.silo2 = np.array([8.9, -4.8, 1.57])
        self.silo3 = np.array([9.65, -4.8, 1.57])
        self.silo4 = np.array([10.4, -4.8, 1.57])
        self.silo5 = np.array([11.15, -4.8, 1.57])
        self.ball1 = np.array([8.5, -1.5, 1.57])
        self.ball2 = np.array([10.0, -1.5 , 1.57])
        self.ball3 = np.array([11.0, -1.5 , 1.57])
        self.goaltake_ball = np.array([9.65, -2.0, 1.57])
        # Testing take ball
        self.silo = 0
        self.silo_end = np.array([0.0, 0.0, 0.0])
        # self.goaltake_ball = np.array([3.0, -0.0, 0.0])
        self.state_end = np.array([0.0, 0.0, 0.0])
        self.state_start = np.array([0.0 , 0.0, 0.0])
        self.current_x = -0.00
        self.current_y = 0.00
        self.current_yaw = 0.0
        self.errorX = [self.state_end[0] - self.current_x]
        self.errorY = [self.state_end[1] - self.current_y]
        self.errorYaw = [self.state_end[2] - self.current_yaw]
        self.errorxball = 0.0
        self.erroryball = 0.0
        self.erroryawball = 0.0
        self.errorx = 0.0
        self.errory = 0.0
        self.erroryaw = 0.0
        self.erroryawsilo = 0.0
        self.state = 0
        self.step = 0
        self.laser_refy = 0.0
        
        
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
        self.current = [0.0,0.0,0.0]
        self.distance = 0.0
        self.silo_goal = 0
        self.laserX = 0.0
        self.lasery = 0.0
        self.laser_siloY = 0.0
        self.laser_siloX = 0.0
        self.typeball = None
        self.Ball = 0
        self.vx_ex = 0.0
        self.vy_ex = 0.0
        self.yaw_imu = 0.0
        self.Wx = 0.0
        self.Wy = 0.0
    def External_feedback(self, current):
        self.Wx = current.data[0]
        self.Wy = current.data[1]
        self.Vx_ex = self.Wx*np.cos(self.current_yaw) - self.Wy*np.sin(self.current_yaw)
        self.Vy_ex = self.Wx*np.sin(self.current_yaw) + self.Wy*np.cos(self.current_yaw)
        self.current_x = self.current_x + self.Vx_ex*self.dt
        self.current_y = self.current_y + self.Vy_ex*self.dt
        
    def Encoder_feedback(self, W):
        self.W1 = W.data[0]
        self.W2 = W.data[1]
        self.W3 = W.data[2]
        self.W4 = W.data[3]
    def laser_callback(self, laser):
        self.laserX = laser.data[0]
        self.laserY = laser.data[1]
    
    def Imu_callback(self, imu_msg):
        self.yaw_imu = imu_msg.z
        if (self.yaw_imu > 2*np.pi or self.yaw_imu < -2*np.pi):
            self.yaw_imu = 0.0
        if (self.yaw_imu >= np.pi):
            self.yaw_imu -= 2*np.pi
        elif (self.yaw_imu <= -np.pi):
               self.yaw_imu += 2*np.pi
        self.yaw_imu = -1* self.yaw_imu
        self.current_yaw = self.yaw_imu
    def subscrib_manual(self, V):
        self.Vx = V.linear.x
        self.Vy = V.linear.y
        self.Omega = V.angular.z
    def subscrib_mode(self, mode):
        self.mode_status = mode.data
    
    def ball_callback(self, ball):
        self.distance = ball.data[0] 
        yaw = -1*ball.data[1]*np.pi/180 
        self.x = self.distance*np.cos(self.yaw)
        self.y = self.distance*np.sin(self.yaw)
        self.yaw = yaw 
        if (self.distance <= 0.32 and self.distance >= 0.2 and self.state == 5):
            time.sleep(1.5)
            self.state = 6