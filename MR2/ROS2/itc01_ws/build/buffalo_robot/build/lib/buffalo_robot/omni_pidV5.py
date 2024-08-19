import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Float32MultiArray,Float32,Int8MultiArray,String,UInt16
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
        self.encoder_sub = self.create_subscription(Float32MultiArray, '/encoder', self.Encoder_feedback, 10)
        self.laser_sub = self.create_subscription(Float32MultiArray, '/laser', self.laser_callback, 10)
        self.sub_ball = self.create_subscription(Float32MultiArray, '/ball_distance', self.ball_callback, 10)
        self.sub_typeball = self.create_subscription(String, '/ball_class', self.balltype_callback, 10)
        self.sub_imu = self.create_subscription(Vector3, "/imu/data", self.Imu_callback, 20)
        ###**** PUBLISHER ****#####
        self.input_pub = self.create_publisher(Float32MultiArray, '/control', 10)
        self.state_pub = self.create_publisher(UInt16, '/state', 10)
        self.ball_count_pub = self.create_publisher(Int8MultiArray, '/total_ball_count', self.ball_count_callback, 10)
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
        self.Kd = np.array([0.008, 0.008, 0.008])
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
        self.pid_Y= PIDController(self.Kp[1], self.Ki[1], self.Kd[1], self.dt, self.alpha, -1.2, 1.2, -1.2, 1.2)
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
        self.goal3 = np.array([9.65, -1.5, 1.57])
        self.goal4 = np.array([9.65, -1.5, 1.57])
        self.goalball = np.array([0.0,0.0,0.0])
        self.silo1 = np.array([8.15, -4.38, 1.57])
        self.silo2 = np.array([8.9, -4.8, 1.57])
        self.silo3 = np.array([9.65, -4.8, 1.57])
        self.silo4 = np.array([10.4, -4.8, 1.57])
        self.silo5 = np.array([11.15, -4.8, 1.57])
        self.ball1 = np.array([8.5, -1., 1.57])
        self.ball2 = np.array([10.0, -1. , 1.57])
        self.ball3 = np.array([11.0, -1. , 1.57])
        self.goaltake_ball = np.array([9.65, -2.0, 1.57])
        # Testing take ball
        self.silo = 0
        self.silo_end = np.array([0.0, 0.0, 0.0])
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
        self.take_ball = 0
        self.data_ball_store = [[0 for _ in range(5)] for _ in range(3)]
        self.robot_move_silo = [0, 0, 0, 0, 0]
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
        # self.current_x = current.data[0]
        # self.current_y = current.data[1]
        # # self.current_yaw = current.data[2]
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
        # q1 = imu_msg.orientation.x
        # q2 = imu_msg.orientation.y
        # q3 = imu_msg.orientation.z
        # q4 = imu_msg.orientation.w
        # siny_cosp = 2 * (q4*q3 + q1*q2)
        # cosy_cosp = 1 - 2 * (q2*q2+q3*q3*q3)
        # self.yaw_imu = -1*np.arctan2(siny_cosp, cosy_cosp) 
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
        self.mode_status = "manual"
    def subscrib_mode(self, mode):
        self.mode_status = mode.data
    
    def ball_callback(self, ball):
        if (self.typeball == "no_ball"):
            self.distance = 0.0
            yaw = 0.0
        else:
            self.distance = ball.data[0] + 0.05
            yaw = -1*ball.data[1]*np.pi/180 + 0.05
        self.x = self.distance*np.cos(self.yaw)
        self.y = self.distance*np.sin(self.yaw)
        self.yaw = yaw 
        if (self.distance <= 0.32 and self.distance >= 0.1 and self.state == 5):
            time.sleep(1.0)
            self.state = 6
    def ball_count_callback(self, count):
        ball_red = count.data[0]
        ball_blue = count.data[1]
        total_ball = count.data[2]
        if (self.silo == 1 and self.state == 10):
            self.data_ball_store[0][0] = "silo1"
            self.data_ball_store[1][0] = ball_red
            self.data_ball_store[2][0] = ball_blue
            self.data_ball_store[3][0] = total_ball
        
        elif (self.silo == 2 and self.state == 10):
            self.data_ball_store[0][1] = "silo2"
            self.data_ball_store[1][1] = ball_red
            self.data_ball_store[2][1] = ball_blue
            self.data_ball_store[3][1] = total_ball
        
        elif (self.silo == 3 and self.state == 10):
            self.data_ball_store[0][2] = "silo3"
            self.data_ball_store[1][2] = ball_red
            self.data_ball_store[2][2] = ball_blue
            self.data_ball_store[3][2] = total_ball
  
        elif (self.silo == 4 and self.state == 10):
            self.data_ball_store[0][3] = "silo4"
            self.data_ball_store[1][3] = ball_red
            self.data_ball_store[2][3] = ball_blue
            self.data_ball_store[3][3] = total_ball
        
        elif (self.silo == 5 and self.state == 10):
            self.data_ball_store[0][4] = "silo5"
            self.data_ball_store[1][4] = ball_red
            self.data_ball_store[2][4] = ball_blue
            self.data_ball_store[3][4] = total_ball
            
        self.get_logger().info(f'data ball in silo store: {self.data_ball_store}')
        # if (self.total_count <=2 and self.state == 10):
            
        # elif (self.total_count >= 3 and self.state == 10):
        #     ball_silo.data = "No"
    def balltype_callback(self, type):
        self.typeball = type.data
    def Calculate_position(self):
        curr_timeball = time.time() 
        if (self.current_x >= 8.0 and self.current_x < 12.0 and self.current_y >= -2.1 and self.current_y < 0.5 and self.state == 5):
            self.pid_X= PIDController(0.7, self.Ki[0], self.Kd[0], self.dt, self.alpha, -0.9, 0.9, -0.9, 0.9)
            self.pid_Y= PIDController(0.7, self.Ki[1], self.Kd[1], self.dt, self.alpha, -0.9, 0.9, -0.9, 0.9)
            self.pid_Yaw= PIDController(1.5, self.Ki[2], self.Kd[2], self.dt, self.alpha, -0.7, 0.7, -0.7, 0.7)
        elif(self.current_x >= 7.2 and self.current_x <= 12.0 and self.current_y >= -10.0 and self.current_y < 0.5 and self.state > 5) :
            self.pid_X= PIDController(1.5, self.Ki[0], self.Kd[0], self.dt, self.alpha, -1.0, 1.0, -1.0, 1.0)
            self.pid_Y= PIDController(1.5, self.Ki[1], self.Kd[1], self.dt, self.alpha, -1.0, 1.0, -1.0, 1.0)
            self.pid_Yaw= PIDController(1.5, self.Ki[2], self.Kd[2], self.dt, self.alpha, -1.0, 1.0, -1.0, 1.0)
        self.errorX.append(self.state_end[0] - self.current_x)
        self.errorY.append(self.state_end[1] - self.current_y)
        self.errorYaw.append(self.state_end[2] - self.current_yaw)

        self.output_vx = self.pid_X.calculate_pid(self.errorX)
        self.output_vy = self.pid_Y.calculate_pid(self.errorY)
        self.output_omega = self.pid_Yaw.calculate_pid(self.errorYaw)

        self.vx = self.output_vx*np.cos(self.current_yaw)  + self.output_vy*np.sin(self.current_yaw)
        self.vy = -self.output_vx*np.sin(self.current_yaw) + self.output_vy*np.cos(self.current_yaw)
        self.omega = self.output_omega
        
        if (self.current_y >=-2.1 and self.current_y <=0.1 and abs(self.erroryawball) >= 0.25 and self.state == 5):
            self.vx = 0.0
            self.vy = 0.0
            self.omega = self.output_omega
        if ((self.current_y >= 0.01 or (self.current_x >= 7.5 and self.current_x <= 8.0) or (self.current_x >= 11.8 and self.current_x <= 12.5)) and self.typeball == "no_ball" and (curr_timeball - self.prev_timeball)>2.0 and self.state == 5):
            self.mode_status = "manual"
            if (self.current_yaw >=0.5 and self.current_yaw <= 3.1415):
                self.Vx = -0.5
            elif (self.current_yaw >= -3.1415 and self.current_yaw < 0.5):
                self.Vx = 0.5
            self.state = 5
            self.prev_timeball = curr_timeball
        elif (self.current_y <=-1.5 and self.mode_status== "manual" and self.state == 5):
            self.Vx = 0.0
            self.state = 5
            self.state_end = self.goaltake_ball
            self.mode_status = "auto"
        if ((self.current_y >= -2.2 and self.current_y<= 0.5) and  self.typeball == "no_ball" and (curr_timeball - self.prev_timeball)>2 and self.state == 5):
            self.mode_status = "manual"
            ball = [1,2]
            Ball = np.random.choice(ball)
            if (Ball == 1):
                self.Omega = -0.5
            elif (Ball == 2):
                self.Omega = 0.5
            self.prev_timeball = curr_timeball
        elif ( self.typeball == "red ball" and self.mode_status == "manual" and self.state == 5 and (self.Ball == 1 or self.Ball ==2 or self.Ball ==3)):
            self.Omega = 0.0
            self.state = 5
            self.state_end = self.goalball
            self.mode_status = "auto"
            self.Ball = 0
        if (self.mode_status == "manual"):
            V = Float32MultiArray()
            W = self.omni.inverse_kinematic(self.Vx,self.Vy,self.Omega,"numpy")
            V.data = [W[0],W[1],W[2],W[3]]  
            self.input_pub.publish(V)
            self.get_logger().info('Velocity_manual : "[%f, %f, %f, %f]"' % (W[0], W[1], W[2], W[3]))
        elif (self.mode_status == "auto"):
            V = Float32MultiArray()
            W = self.omni.inverse_kinematic(self.vx,self.vy,self.omega,"numpy")
            if (W[0] < 1.0 and W[0] > - 1.0):
                W[0] = 0.0
            if (W[1] < 1.0 and W[1] > - 1.0):
                W[1] = 0.0
            if (W[2] < 1.0 and W[2] > - 1.0):
                W[2] = 0.0
            if (W[3] < 1.0 and W[3] > - 1.0):
                W[3] = 0.0
            V.data = [W[0],W[1],W[2],W[3]] 
            self.input_pub.publish(V)

            
    def calculate_goal(self):
        # curr_time = time.time() 
        state_msg = UInt16()
        self.errorx = self.state_end[0] - self.current_x
        self.errory = self.state_end[1] - self.current_y
        self.erroryaw = self.state_end[2] - self.current_yaw
        ##** Auto from Area1 to Area2 : FULL OPTION
        if (self.goal == "goal1" and self.state == 0):
            self.state_end = self.goal0
            self.state = 1
        elif (abs(self.errorx)<=0.1 and self.state == 1):
            self.state_end = self.goal1
            self.state = 2
        elif (abs(self.errory)<=0.1 and self.state == 2):
            error_lasery = -self.laserY+1.37
            if (abs(error_lasery)>=0.08):
                self.state_end = [self.current_x,error_lasery+self.current_y,0.0]
            elif (abs(error_lasery)<0.08):
                self.state_end = [self.goal2[0],self.current_y,self.goal2[2]]
                self.state = 3
        elif (abs(self.errorx)<=0.1 and self.state == 3):
            self.state_end = self.goal3
            self.state = 4
        elif (abs(self.errory)<=0.1 and self.state == 4):
            self.state_end = self.goal4
            self.state = 5
        # Start Play on Area3 take ball    
        elif (self.current_x > 8.0 and self.current_x < 12.0 and self.current_y >= -2.1 and self.current_y < 0.1 and self.state == 5 and self.typeball == "red ball"):
            
            self.goalball[0] = self.x*np.cos(self.current_yaw) - self.y*np.sin(self.current_yaw) + self.current_x
            self.goalball[1] = self.x*np.sin(self.current_yaw) + self.y*np.cos(self.current_yaw) + self.current_y
            self.goalball[2] = self.yaw + self.current_yaw
             
            if (self.goalball[2] > 2*np.pi or self.goalball[2] <-2*np.pi):
                self.goalball[2]=0.0
            if (self.goalball[2] >= np.pi):
                self.goalball[2] -=2*np.pi
            elif (self.goalball[2] <= -np.pi):
                self.goalball[2] +=2*np.pi
            self.erroryawball = self.goalball[2] - self.current_yaw
            self.state_end = self.goalball
            self.erroryball = self.goalball[1] - self.current_y
            self.errorxball = self.goalball[0] - self.current_x
            self.get_logger().info('Goal : "[%f, %f, %f]"' % (self.goalball[0], self.goalball[1], self.goalball[2]))
            self.take_ball = 1
        elif (self.state == 6 ):
            self.step = 1
            self.state_start = [self.current_x, self.current_y, self.current_yaw]
            self.state_end = [self.current_x,self.current_y,1.57]
            self.erroryawsilo = self.state_end[2] - self.current_yaw
            if (abs(self.erroryawsilo)<=0.3 and self.laserY >= 1.8 and self.step == 1):
                silo = [1,2,3]
                self.silo = np.random.choice(silo)
                if (self.silo == 1):
                    # self.state_end = [8.15,-3.0,1.57]
                    self.robot_move_silo[1] +=  1
                    self.state_end = [8.9,-3.0,1.57]
                    self.silo_end = self.silo2
                    self.state = 7
                    self.step = 0
                elif (self.silo == 2):
                    self.robot_move_silo[1] += 1
                    self.state_end = [8.9,-3.0,1.57]
                    self.silo_end = self.silo2
                    self.state = 7
                    self.step = 0
                elif (self.silo == 3):
                    self.robot_move_silo[2] += 1
                    self.state_end = [9.65,-3.0,1.57]
                    self.silo_end = self.silo3
                    self.state = 7
                    self.step = 0
            elif (abs(self.erroryaw)<=0.3 and self.laserY < 1.8 and self.step == 1):
                silo = [3,4,5]
                
                self.silo = np.random.choice(silo)
                if (self.silo == 3):
                    self.robot_move_silo[2] += 1
                    self.state_end = [9.65,-3.0,1.57]
                    self.silo_end = self.silo3
                    self.state = 7
                    self.step = 0
                elif (self.silo == 4):
                    self.robot_move_silo[3] += 1
                    self.state_end = [10.3,-3.0,1.57]
                    self.silo_end = self.silo4
                    self.state = 7
                    self.step = 0
                elif (self.silo == 5):
                    self.robot_move_silo[3] += 1
                    self.state_end = [10.4,-3.0,1.57]
                    self.silo_end = self.silo4
                    self.state = 7
                    self.step = 0
        elif (abs(self.errorx)<=0.1  and self.state == 7 and self.step==0):
            self.state_end = self.silo_end
            self.state = 8
            self.get_logger().info('Goal_Silo :"%d"' %self.silo)
    
        elif (abs(self.errory)<=0.1  and self.state == 8):
            self.state_start = [self.current_x, self.current_y, self.current_yaw]
            if (self.robot_move_silo[1] > 3):
                self.silo = 3
            if (self.robot_move_silo[2] > 3):
                self.silo = 4
            if (self.robot_move_silo[3] > 3):
                self.silo = 2
            if (self.silo == 1):
                self.laser_siloY =  -2.549+self.silo2[0] # laser_ref + Y_ref 3.294+8.65
                self.laser_refy = -2.549
            elif (self.silo == 2):
                self.laser_siloY = -2.549+self.silo2[0]
                self.laser_refy = -2.549
            elif (self.silo == 3):
                self.laser_siloY = -1.8+self.silo3[0]
                self.laser_refy = -1.78
            elif (self.silo == 4):
                self.laser_siloY = -1.02+self.silo4[0]
                self.laser_refy = -1.02
            elif (self.silo == 5):
                self.laser_siloY = -1.02+self.silo4[0]
                self.laser_refy = -1.02
            self.state_end = [self.laserY+self.laser_siloY, -self.laserX + 0.23+self.current_y, 1.57]
            self.get_logger().info('Goal_Silo :"%d"' %self.silo)
            self.state = 9
        
        elif ((abs(self.errory)<=0.25) and (self.laserX >= 0.1 and self.laserX <= 1.1)  and self.state == 9):
            error_laserx = -self.laserX + 0.21
            error_lasery = self.laserY + self.laser_refy
            self.get_logger().info('erorrlaserx: %f, errorrlaeserY: %f' % (error_laserx, error_lasery))
            if ((abs(error_laserx)>=0.05 or (abs(error_lasery)>=0.05)) and self.state == 9):
                self.state_end = [self.current_x+error_lasery, error_laserx+self.current_y, 1.57]
            elif ((abs(error_laserx)<=0.05 and (abs(error_lasery)<=0.06)) and (self.state == 9)):
                self.state = 10
                state_msg.data = self.state
                self.state_pub.publish(state_msg)
                time.sleep(2.0)
                self.state_end = self.goaltake_ball
                self.state = 5
        print(self.state)
        state_msg.data = self.state
        self.state_pub.publish(state_msg)
        self.get_logger().info('Goal : "[%f, %f, %f]"' % (self.state_end[0], self.state_end[1], self.state_end[2]))
        self.get_logger().info('current : "[%f, %f, %f]"' % (self.current_x, self.current_y, self.current_yaw))
        self.get_logger().info('mode : %s ' % (self.mode_status))
def main(args=None):
    rclpy.init(args=args)
    pid_bf = Buffalo_PID()
    rclpy.spin(pid_bf)
    pid_bf.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()