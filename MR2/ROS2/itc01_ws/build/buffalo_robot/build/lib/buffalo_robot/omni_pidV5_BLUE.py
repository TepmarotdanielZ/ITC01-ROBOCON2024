import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Float32MultiArray,Float32,Int8,String,UInt16,UInt8MultiArray,Int8MultiArray
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
        self.ball_aready_sub = self.create_subscription(UInt8MultiArray, '/ball_already',self.ball_already_callback, 10)
        # self.sub_imu = self.create_subscription(Vector3, "/imu/data", self.Imu_callback, 20)
       
        ###**** PUBLISHER ****#####
        self.input_pub = self.create_publisher(Float32MultiArray, '/control', 10)
        self.state_pub = self.create_publisher(UInt16, '/state', 10)
        ###*** TIMER ****####
        timer = 0.01
        self.output_timer = self.create_timer(0.01,self.Calculate_position)
        self.ball_timer = self.create_timer(timer,self.calculate_goal)
        
        ###*** VARIABLE PARAMETER ***###
        ### Parameter of Robot
        self.R = 0.06
        self.Lx = 0.30/2
        self.Ly = 0.30/2 
        self.omni=Omni_model(self.R,self.Lx,self.Ly)
        self.sampling_time = 0.01
        
        ## Variable PID Auto
        self.dt = 0.01
        self.Kp = np.array([2.0, 2.0, 1.0])
        self.Ki = np.array([0.0001, 0.0001, 0.0001])
        self.Kd = np.array([0.008, 0.008, 0.008])
        self.integral_min = -1.6
        self.integral_max = 1.6
        self.output_min = -1.6
        self.output_max = 1.6
        self.integral_min_Angular = -1.0
        self.integral_max_Angular = 1.0
        self.output_min_Angular = -1.0
        self.output_max_Angular = 1.0
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
        # self.prev_timeball1 = time.time()
        
        self.sampling_time = 0.01
        self.mode_status = "auto"
        self.goal = None
        self.X_start = 0.0
        self.Y_start = 0.0
        self.Yaw_start = 0.0
        self.Goal = np.array([0.0, -0.0, 0.0])
        # GOAL FIRST START
        self.goal0 = np.array([6.1, -0.1, 0.0])
        self.goal1 = np.array([6.1, 3.5, 0.0])
        self.goal2 = np.array([9.5, 3.5, 0.0])
        self.goal3 = np.array([10.0, 1.5, -1.57])
        self.goal4 = np.array([10.0, 1.5, -1.57])
        
        # GOAL RETRY START
        self.goal0_retry = [0.8, 1.2, 0.0]
        self.goal1_retry = [0.8, 3.5, 0.0]
        self.goal2_retry = [4.475, 3.5, 0.0]
        self.goal3_retry = [4.475, 1.5, -1.57]
        self.goal4_retry = [4.475, 1.5, -1.57]
        
        # GOAL BALL
        self.goalball = np.array([0.0,0.0,0.0])
        self.goaltake_ball = np.array([9.65, 1.5, -1.57])
        
        # VARIABLE DESIRESD POSITION
        self.silo = 0
        self.silo_count = [0, 0, 0, 0, 0, 0]
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
        self.take_ball = 0
        self.data_ball_store = [[0 for _ in range(5)] for _ in range(4)]
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
        self.check_three_ball = 0
        self.ball_aready = 0
        self.typeball = None
        self.Ball = 0
        self.vx_ex = 0.0
        self.vy_ex = 0.0
        self.yaw_imu = 0.0
        self.Wx = 0.0
        self.Wy = 0.0
        self.state_limitvx = 0
        self.state_switch = 0
        self.state_move = 0
        self.state_yaw = 0
        self.state_count = 0
        # PARAMETER LASER
        self.goal_limitball = [0.0, 0.0]
        self.laser_refxsilo = 0.01
        self.laser_refxcheckball = 0.05
        self.laser_refysilo = [0.0, 3.13, 2.40, 1.63, 0.89, 0.125]
        self.laser_refywall = [0.0, 2.89, 2.15, 1.41, 0.635]
        self.laser_refxwall = [1.40, 1.75]
        self.laser_bridgeY = 1.35 # when battery full is 1.65
        self.laserX = 0.0
        self.lasery = 0.0
        self.laser_siloY = 0.0
        self.laser_siloX = 0.0
        self.laser_refy = 0.0
        self.button = 0
    def External_feedback(self, current):
        self.Wx = current.data[0]
        self.Wy = current.data[1]
        self.yaw_imu = current.data[2]
        
    def Encoder_feedback(self, W):
        self.W1 = W.data[0]
        self.W2 = W.data[1]
        self.W3 = W.data[2]
        self.W4 = W.data[3]
    def laser_callback(self, laser):
        self.laserX = laser.data[0]
        self.laserY = laser.data[1]
      
    def ball_already_callback(self, ball):
        self.ball_aready = ball.data[0]
        self.button = ball.data[1]
        self.check_three_ball = ball.data[2]
        self.get_logger().info('Ball_inside :"%d"' %self.ball_aready)
        if (self.ball_aready == 1 and self.state == 5):
            self.state = 6
            self.mode_status = "auto"
        if (self.button == 1):
            self.goal = "success"
        elif (self.button == 2):
            self.goal = "retry"
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
            self.distance = ball.data[0] 
            yaw = -1*ball.data[1]*np.pi/180 + 0.06
        self.x = self.distance*np.cos(self.yaw)
        self.y = self.distance*np.sin(self.yaw)
        self.yaw = yaw 
        if (self.distance <= 0.31 and self.distance >= 0.2 and self.state == 5):
            time.sleep(0.7)
            self.mode_status = "auto"
            self.state = 6
        
    def balltype_callback(self, type):
        self.typeball = type.data
    def Calculate_position(self):
        # curr_timeball1 = time.time()
        self.Vx_ex = self.Wx*np.cos(self.yaw_imu) - self.Wy*np.sin(self.yaw_imu)
        self.Vy_ex = self.Wx*np.sin(self.yaw_imu) + self.Wy*np.cos(self.yaw_imu)
        self.current_x = self.current_x + self.Vx_ex*self.dt
        self.current_y = self.current_y + self.Vy_ex*self.dt
        self.current_yaw = self.yaw_imu
        if ((self.state == 5)):
            self.pid_X= PIDController(0.6, self.Ki[0], self.Kd[0], self.dt, self.alpha, -0.9, 0.9, -0.9, 0.9)
            self.pid_Y= PIDController(0.6, self.Ki[1], self.Kd[1], self.dt, self.alpha, -0.9, 0.9, -0.9, 0.9)
            self.pid_Yaw= PIDController(2.0, self.Ki[2], self.Kd[2], self.dt, self.alpha, -1.3, 1.3, -1.3, 1.3)
         # When catch already and to take ball
        elif(((self.state >= 6 and self.state<=9))):
            self.pid_X= PIDController(1.5, self.Ki[0], self.Kd[0], self.dt, self.alpha, -1.2, 1.2, -1.2, 1.2)
            self.pid_Y= PIDController(1.5, self.Ki[1], self.Kd[1], self.dt, self.alpha, -1.2, 1.2, -1.2, 1.2)
            self.pid_Yaw= PIDController(1.8, self.Ki[2], self.Kd[2], self.dt, self.alpha, -1.5, 1.5, -1.5, 1.5)
        # When Find silo
        elif((self.state >= 10 and self.state<=15)):
            self.pid_X= PIDController(2.2, self.Ki[0], self.Kd[0], self.dt, self.alpha, -1.0, 1.0, -1.0, 1.0)
            self.pid_Y= PIDController(2.2, self.Ki[1], self.Kd[1], self.dt, self.alpha, -1.0, 1.0, -1.0, 1.0)
            self.pid_Yaw= PIDController(1.2, self.Ki[2], self.Kd[2], self.dt, self.alpha, -0.7, 0.7, -0.7, 0.7)
        self.errorX.append(self.state_end[0] - self.current_x)
        self.errorY.append(self.state_end[1] - self.current_y)
        self.errorYaw.append(self.state_end[2] - self.current_yaw)

        self.output_vx = self.pid_X.calculate_pid(self.errorX)
        self.output_vy = self.pid_Y.calculate_pid(self.errorY)
        self.output_omega = self.pid_Yaw.calculate_pid(self.errorYaw)

        self.vx = self.output_vx*np.cos(self.current_yaw)  + self.output_vy*np.sin(self.current_yaw)
        self.vy = -self.output_vx*np.sin(self.current_yaw) + self.output_vy*np.cos(self.current_yaw)
        self.omega = self.output_omega
        
        if (self.current_y <=4.5 and self.current_y >=-0.1 and abs(self.erroryawball) >= 0.25 and self.state == 5 and self.typeball == "blue ball"):
            self.vx = 0.0
            self.vy = 0.0
            self.omega = self.output_omega
            
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

        print(self.take_ball)  
    def calculate_goal(self):
        curr_timeball = time.time()
        state_msg = UInt16()
        self.errorx = self.state_end[0] - self.current_x
        self.errory = self.state_end[1] - self.current_y
        self.erroryaw = self.state_end[2] - self.current_yaw
        if (self.ball_aready == 0 and self.state == 6):
            self.state = 5
            self.state_end = self.goaltake_ball
            self.mode_status = "auto"
        if ( self.check_three_ball == 0 and self.state == 11 ):
            self.state = 12
        elif (self.check_three_ball ==1 and self.state == 11 ):
            self.silo -=1
            if (self.silo == 0 ):
                self.silo = 5
            self.state = 22
        ##** Auto from Area1 to Area2 : FULL OPTION
        if (self.goal == "success" ):
            self.goal_limitball = [7.5, 12.5] # [min, max]
            if (self.state == 0):
                self.state_end = self.goal0
                self.state = 1
            elif (abs(self.errorx)<=0.2 and self.state == 1):
                self.state_end = self.goal1
                self.state = 2
            elif (abs(self.errory)<=0.2 and self.state == 2):
                error_lasery = self.laserY-self.laser_bridgeY
                if (abs(error_lasery)>=0.15):
                    self.state_end = [self.current_x,error_lasery+self.current_y,0.0]
                elif (abs(error_lasery)<0.15):
                    self.state_end = [self.goal2[0],self.current_y,self.goal2[2]]
                    self.state = 3
            elif (abs(self.errorx)<=0.2 and self.state == 3):
                self.state_end = self.goal3
                self.state = 4
            elif (abs(self.errory)<=0.2 and self.state == 4):
                self.state_end = self.goal4
                self.state = 5
                self.goal = None
        
        ##*** WHEN ROBOT RETRY ***##
        elif (self.goal == "retry" ):
            self.goal_limitball = [1.5, 6.5] # [min, max]
            if (self.state == 0):
                self.state_end = self.goal0_retry
                self.state = 1
            elif (abs(self.errorx)<=0.05 and self.state == 1):
                self.state_end = self.goal1_retry
                self.state = 2
            elif (abs(self.errory)<=0.05 and self.state == 2):
                error_lasery = self.laserY-self.laser_bridgeY
                if (abs(error_lasery)>=0.15):
                    self.state_end = [self.current_x,error_lasery+self.current_y,0.0]
                elif (abs(error_lasery)<0.15):
                    self.state_end = [self.goal2_retry[0],error_lasery+self.current_y,self.goal2_retry[2]]
                    self.state = 3
            elif (abs(self.errorx)<=0.1 and self.state == 3):
                self.state_end = self.goal3_retry
                self.state = 4
            elif (abs(self.errory)<=0.1 and self.state == 4):
                self.state_end = self.goal4_retry
                self.state = 5
                self.goal = None
        #if laser=1.5 robot is stop 1s to open webcame to count ball in silo      
        ###*** START PLAY Ball IN AREA3  ***###
        # WHEN ROBOT CATCH BALL
        elif ((self.current_x >= self.goal_limitball[0] and self.current_x < self.goal_limitball[1])  and (self.current_y <= 4.5 and self.current_y >= -0.1) and self.state == 5 and self.typeball == "blue ball"):
            self.mode_status = "auto"
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
            self.take_ball = 0
            self.state_yaw = 0
        elif ((self.distance<=0.40 and self.distance>0.25) and (curr_timeball-self.prev_time)>2.0 and self.state == 5 and self.mode_status == "auto"):
            self.mode_status = "manual"
            self.Omega = -1.5
            self.state = 20
            self.prev_time = curr_timeball
        elif (self.state == 20 and (curr_timeball-self.prev_time)>1.0):
            self.mode_status = "auto"
            self.state = 5
            self.prev_time = curr_timeball
        #When NOT SEE BALL
        elif ((self.current_y <= 4.8)  and (curr_timeball - self.prev_timeball)>2.0 and self.state == 5  and  self.typeball == "no_ball" and self.state_yaw ==0 ):
            self.mode_status = "manual"
            self.Omega = 1.5
            self.state_yaw = 1
            self.state_count = 0
            self.prev_timeball = curr_timeball 
        elif ((self.current_yaw>=0.0 and self.current_yaw <=0.4) and self.mode_status == "manual" and  self.typeball == "no_ball" and self.state == 5  and self.state_yaw == 1):  
            self.mode_status = "manual"
            self.Omega = -1.5
            self.prev_timeball = curr_timeball
        elif ((self.current_yaw>= 2.5 and self.current_yaw <= 3.0) and self.mode_status == "manual" and  self.typeball == "no_ball" and self.state == 5  and self.state_yaw == 1): 
            self.mode_status = "manual"
            self.Omega = 1.5
            self.state_count +=1
            self.state_move ==0
            self.prev_timeball = curr_timeball
        elif (self.state_count == 1 and self.state == 5 and self.typeball == "no_ball" and self.state_move ==0):
            self.Omega = 0.0
            self.mode_status = "auto"
            self.state_end = [self.current_x+0.5, self.current_y-0.3, -1.57]
            self.state_count = 0
            self.state_move = 1
            self.state_yaw = 0
        elif (self.state_count == 2 and self.state == 5 and self.typeball == "no_ball" and self.state_move == 1):
            self.Omega = 0.0
            self.mode_status = "auto"
            self.state_end = [self.current_x-0.5, self.current_y-1.0, -1.57]
            self.state_count = 0
            self.state_move = 0
            self.state_yaw = 0
        
        # WHEN ROBOT TAKE BALL AREADY
        elif (self.state == 6 and self.mode_status == "auto"):
            if (self.current_x >= self.goal_limitball[0] and self.current_x <= self.goal_limitball[0] +2.5):
                self.state_end = [self.current_x+0.7, self.current_y+3.5, -0.75]
            elif ((self.laserY >=0.02 and self.laserY<=1.0 and self.current_x>=self.goal_limitball[1] -1.5) or (self.current_x>=self.goal_limitball[1]-2.0 and self.current_x<=self.goal_limitball[1])):
                self.state_end = [self.current_x-0.8, self.current_y+3.5, -0.75]
            else:
                self.state_end = [self.current_x-0.5, self.current_y+3.5, -0.75]
            self.state = 7
        elif (abs(self.errory)<=0.3 and abs(self.erroryaw)<=0.1 and self.state == 7):
            self.state_end = [self.current_x, self.current_y, -1.57]
            self.state = 17
        # WHEN ROBOT CHECK LASER TO MOVE SILO POSITION_X IN ROBOT NAER
        # elif (abs(self.erroryaw)<=0.1 and abs(self.errory)<=0.1 and not((self.laserX>=self.laser_refxwall[0] and self.laserX<=self.laser_refxwall[1])) and self.state == 17):
        #     if (self.laserY >= self.laser_refysilo[3]):
        #         silo = [1,2,3]
        #         self.silo = np.random.choice(silo)
        #         self.silo_count[self.silo] += 1
        #     elif (self.laserY <= self.laser_refysilo[3]):
        #         silo = [3,4]
        #         self.silo = np.random.choice(silo)
        #         self.silo_count[self.silo] += 1
        #     self.laser_refy = self.laser_refysilo[self.silo]
        #     self.state = 8
        # elif (abs(self.erroryaw)<=0.1 and abs(self.errory)<=0.1 and (self.laserX>=self.laser_refxwall[0] and self.laserX<=self.laser_refxwall[1]) and self.state == 17):
        #     if (self.laserY >= self.laser_refywall[3]):
        #         silo = [1,2,3]
        #         self.silo = np.random.choice(silo)
        #         self.silo_count[self.silo] += 1
        #     elif (self.laserY <= self.laser_refywall[3]):
        #         silo = [3,4]
        #         self.silo = np.random.choice(silo)
        #         self.silo_count[self.silo] += 1
        #     self.state = 8
            # self.laser_refy = self.laser_refywall[self.silo]
        elif (abs(self.erroryaw)<=0.1 and abs(self.errory)<=0.1  and self.state == 17):
            if (self.laserY >= self.laser_refysilo[3]):
                silo = [1,2,3]
                self.silo = np.random.choice(silo)
                self.silo_count[self.silo] += 1
            elif (self.laserY <= self.laser_refysilo[3]):
                silo = [3,4,5]
                self.silo = np.random.choice(silo)
                self.silo_count[self.silo] += 1
            self.laser_refy = self.laser_refysilo[self.silo]
            self.state = 8
        # WHEN ROBOT START MOVE TO SILO
        elif (self.state == 8 ):
            error_lasery = self.laserY - self.laser_refy
            if (abs(error_lasery)>= 0.2):
                self.state_end = [self.current_x+error_lasery, self.current_y, -1.57]
            elif (abs(error_lasery)< 0.2 ):
                self.state_end = [self.current_x, 3.8, -1.57]
                self.state = 9
            self.get_logger().info('Goal_Silo :"%d"' %self.silo)
        
        elif ((self.state == 9 and (not(self.laserX >= 0.01 and self.laserX <= 1.4)))):
            self.state_end = [self.current_x, self.current_y+1.0, -1.57]
            self.state == 10
       
        #  CHECK  Threee Ball
        elif ((self.laserX >= 0.001 and self.laserX <= 1.4) and (self.state == 10 or self.state==9)):
            error_laserx_py = self.laserX - self.laser_refxcheckball
            error_lasery_px = self.laserY - self.laser_refysilo[self.silo]
            if (abs(error_lasery_px)>=0.05  and abs(error_laserx_py)>=0.04):
                self.state_end = [self.current_x+error_lasery_px, error_laserx_py+self.current_y, -1.57]
            elif (abs(error_lasery_px)<0.05  and abs(error_laserx_py)<0.04):
                self.state = 11
                time.sleep(0.8)
        elif ((self.state == 22 )):
            error_laserx_py = self.laserX - self.laser_refxcheckball
            error_lasery_px = self.laserY - self.laser_refysilo[self.silo]
            if (abs(error_lasery_px)>=0.04 ):
                self.state_end = [self.current_x+error_lasery_px, error_laserx_py+self.current_y, -1.57]
            elif (abs(error_lasery_px)<0.04  and abs(error_laserx_py)<0.04):
                self.state = 11
                time.sleep(0.8)
        # WHEN ROBOT CHECK POSITION_Y AND SHOOT BALL TO SILO
        elif ((self.laserX >= 0.01 and self.laserX <= 2.2)  and (self.state == 12)):
            error_laserx_py = self.laserX - self.laser_refxsilo
            error_lasery_px = self.laserY - self.laser_refysilo[self.silo]
            self.get_logger().info('erorrlaserx: %f, errorrlaeserY: %f' % (error_laserx_py, error_lasery_px))
            if ((abs(error_laserx_py)>=0.02 and (abs(error_lasery_px)>=0.04)) and self.state == 12):
                self.state_end = [self.current_x+error_lasery_px, error_laserx_py+self.current_y, -1.57]
            elif ((abs(error_laserx_py)<0.02 and (abs(error_lasery_px)< 0.04)) and (self.state == 12)):
                self.state = 13
                state_msg.data = self.state
                self.state_pub.publish(state_msg)
                time.sleep(1.0)
                errorsilo3 = self.laserY - self.laser_refysilo[3]
                self.goaltake_ball = [self.current_x+errorsilo3, 2.3, -1.57]
                self.state_end = self.goaltake_ball
                self.state = 14          
        # # WHEN ROBOT RETURN FROM SILO TO CATCH BALL AGAIN
        
        # WHEN ROBOT RETURN FROM SILO TO CATCH BALL AGAIN
        elif ((self.laserX<=2.1 and self.laserX>=1.7) and self.state == 14 ):
                errorsilo3 = self.laserY - self.laser_refysilo[3]
                self.state_end = [self.current_x+errorsilo3, self.current_y-2.3, -1.57]
                self.state = 15
        elif (abs(self.errory)<=0.5 and self.state == 15):
                self.state = 5
                self.take_ball = 0
                self.state_move = 0
                self.state_yaw = 0
        state_msg.data = self.state
        self.state_pub.publish(state_msg)
        self.get_logger().info('Goal : "[%f, %f, %f]"' % (self.state_end[0], self.state_end[1], self.state_end[2]))
        self.get_logger().info('current : "[%f, %f, %f]"' % (self.current_x, self.current_y, self.current_yaw))
        self.get_logger().info('mode : %s ' % (self.mode_status))
        self.get_logger().info('Goal_Silo :"%d", State: %d, ball_type: %s' %(self.silo, self.state, self.typeball))
        self.get_logger().info(f'data ball in silo store: {self.silo_count}')
def main(args=None):
    rclpy.init(args=args)
    pid_bf = Buffalo_PID()
    rclpy.spin(pid_bf)
    pid_bf.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()