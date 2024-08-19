import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
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
        self.sub_ball = self.create_subscription(Float32MultiArray, '/ball_distance', self.ball_callback, 10)
        
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
        self.Kp = np.array([1.5, 1.0, 2.0])
        self.Ki = np.array([0.0000, 0.0000, 0.0000])
        self.Kd = np.array([0.008, 0.008, 0.008])
        self.integral_min = -1.0
        self.integral_max = 1.0
        self.output_min = -1.0
        self.output_max = 1.0
        self.integral_min_Angular = -0.5
        self.integral_max_Angular = 0.5
        self.output_min_Angular = -0.5
        self.output_max_Angular = 0.5
        self.alpha = 0.8
        self.pid_X= PIDController(self.Kp[0], self.Ki[0], self.Kd[0], self.dt, self.alpha, self.integral_min, self.integral_max, self.output_min, self.output_max)
        self.pid_Y= PIDController(self.Kp[1], self.Ki[1], self.Kd[1], self.dt, self.alpha, self.integral_min, self.integral_max, self.output_min, self.output_max)
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
        self.Goal = np.array([0.0, 0.0, 0.0])
        # self.goal0 = np.array([6.0, 0.0, 0.0])
        # self.goal1 = np.array([6.3, -3.8, 0.0])
        # self.goal2 = np.array([9.5, -3.8, 0.0])
        # self.goal3 = np.array([9.5, -3.8, 1.57])
        # self.goal4 = np.array([10.5, -1.5, 1.57])
        # self.goalball = np.array([0.0,0.0,0.0])
        # self.silo1 = np.array([8.1, -4.8, 1.57])
        # self.silo2 = np.array([8.9, -4.8, 1.57])
        # self.silo3 = np.array([10.2, -5.5, 1.57])
        # self.silo4 = np.array([10.4, -4.8, 1.57])
        # self.silo5 = np.array([11.15, -4.8, 1.57])
        # self.goaltake_ball = np.array([10.5, -2.0, 1.57])
        # Testing take ball
        self.silo = 0
        self.goal0 = np.array([3.0, -0.0, 0.0])
        self.goalball = np.array([0.0,0.0,0.0])
        self.silo1 = np.array([0.0, 1.53, 0.0])
        self.silo2 = np.array([0.0, 0.78,0.0])
        self.silo3 = np.array([-0.0,0.0,0.0])
        self.silo4 = np.array([0.0,-0.78,0.0])
        # self.silo5 = np.array([0.0,-1.5,0.0])
        self.silo_end = np.array([0.0, 0.0, 0.0])
        self.goaltake_ball = np.array([3.0, -0.0, 0.0])
        self.state_end = np.array([0.0, 0.0, 0.0])
        self.state_start = np.array([0.0 , 0.0, 0.0])
        self.current_x = -0.00
        self.current_y = 0.00
        self.current_yaw = 0.0
        self.errorX = [self.state_end[0] - self.current_x]
        self.errorY = [self.state_end[1] - self.current_y]
        self.errorYaw = [self.state_end[2] - self.current_yaw]
        self.errorx = 0.0
        self.errory = 0.0
        self.erroryaw = 0.0
        self.erroryawball = 0.0
        self.state = 0
        self.step = 0
        
        
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
        
   
    def External_feedback(self, current):
        self.current_x = current.data[0]
        self.current_y = current.data[1]
        self.current_yaw = current.data[2]
        
    def Encoder_feedback(self, W):
        self.W1 = W.data[0]
        self.W2 = W.data[1]
        self.W3 = W.data[2]
        self.W4 = W.data[3]
    def laser_callback(self, laser):
        self.laserX = laser.data[0]
        self.laserY = laser.data[1]
    
    def subscrib_manual(self, V):
        self.Vx = V.linear.x
        self.Vy = V.linear.y
        self.Omega = V.angular.z
    def subscrib_mode(self, mode):
        self.mode_status = mode.data
    
    def ball_callback(self, ball):
        self.distance = ball.data[0] 
        yaw = -1*ball.data[1]*np.pi/180 + 0.04
        self.x = self.distance*np.cos(self.yaw)
        self.y = self.distance*np.sin(self.yaw)
        self.yaw = yaw 
        if (self.distance <= 0.35 and self.distance >= 0.2):
            time.sleep(0.7)
            self.state = 6

    def Calculate_position(self):
        # if (self.current_x > 8.0 and self.current_x < 12.0 and self.current_y >= -1.6 and self.current_y < 0.2):
        #     self.pid_X= PIDController(1.5, self.Ki[0], self.Kd[0], self.dt, self.alpha, -1.0, 1.0, -1.0, 1.0)
        #     self.pid_Y= PIDController(1.0, self.Ki[1], self.Kd[1], self.dt, self.alpha, -1.0, 1.0, -1.0, 1.0)
        #     self.pid_Yaw= PIDController(0.8, self.Ki[2], self.Kd[2], self.dt, self.alpha, -0.7, 0.7, -0.7, 0.7)
        # elif (self.current_x >= 8.0 and self.current_x =< 12.0 and self.current_y > -4.8 and self.current_y <= -2.0):
        #     self.pid_X= PIDController(self.Kp[0], self.Ki[0], self.Kd[0], self.dt, self.alpha, self.integral_min, self.integral_max, self.output_min, self.output_max)
        #     self.pid_Y= PIDController(self.Kp[1], self.Ki[1], self.Kd[1], self.dt, self.alpha, self.integral_min, self.integral_max, self.output_min, self.output_max)
        #     self.pid_Yaw= PIDController(self.Kp[2], self.Ki[2], self.Kd[2], self.dt, self.alpha, self.integral_min_Angular, self.integral_max_Angular, self.output_min_Angular, self.output_max_Angular)
            
        self.errorX.append(self.state_end[0] - self.current_x)
        self.errorY.append(self.state_end[1] - self.current_y)
        self.errorYaw.append(self.state_end[2] - self.current_yaw)

        self.output_vx = self.pid_X.calculate_pid(self.errorX)
        self.output_vy = self.pid_Y.calculate_pid(self.errorY)
        self.output_omega = self.pid_Yaw.calculate_pid(self.errorYaw)

        self.vx = self.output_vx*np.cos(self.current_yaw)  + self.output_vy*np.sin(self.current_yaw)
        self.vy = -self.output_vx*np.sin(self.current_yaw) + self.output_vy*np.cos(self.current_yaw)
        self.omega = self.output_omega
        
        if (self.current_x >=2.8 and self.current_x <=3.2 and abs(self.erroryawball) > 0.3 and self.state ==1):
            self.vx = 0.0
            self.vy = 0.0
            self.omega = self.output_omega
        
        if (self.mode_status == "manual"):
            V = Float32MultiArray()
            W = self.omni.inverse_kinematic(self.Vx,self.Vy,self.Omega,"numpy")
            V.data = [W[0],W[1],W[2],W[3]]  
            self.input_pub.publish(V)
            # self.get_logger().info('Velocity control : "[%f, %f, %f, %f]"' % (W[0], W[1], W[2], W[3]))
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

            # self.get_logger().info('Goal : "[%f, %f, %f]"' % (self.output_vx, self.output_vy, self.output_omega))
    def calculate_goal(self):
         curr_time = time.time() 
         self.errorx = self.state_end[0] - self.current_x
        #  self.errory = self.state_end[1] - self.current_y
        #  self.erroryaw = self.state_end[2] - self.current_yaw
        ##** Auto from Area1 to Area2 : FULL OPTION
        # if (self.goal == "goal1" and self.state == 0):
        #     self.state_end = self.goal0
        #     self.state_start = [self.current_x, self.current_y, self.current_yaw]
        #     self.state = 1
        # elif (self.current_x >(self.goal0[0]-0.1)  and self.current_x < (self.goal0[0]+0.1) and self.state == 1):
        #     self.state_end = self.goal1
        #     self.state_start = [self.current_x, self.current_y, self.current_yaw]
        #     self.state = 2
        # elif (self.current_y < (self.goal1[1]+0.2) and self.current_y > (self.goal1[1]-0.2) and self.state == 2):
        #     self.state_end = [self.goal2[0],-slef.laserY+1.5-3.8,self.goal2[2]]
        #     self.state_start = [self.current_x, self.current_y, self.current_yaw]
        #     self.state = 3
        # elif (self.current_x > (self.goal2[0]-0.1) and self.current_x < (self.goal2[0]+0.1) and self.state == 3):
        #     self.state_end = self.goal3
        #     self.state_start = [self.current_x, self.current_y, self.current_yaw]
        #     self.state = 4
        # elif (self.current_yaw > (self.goal3[2]-0.4) and self.current_yaw < (self.goal3[2]+0.4) and self.state == 4):
        #     self.state_end = self.goal4
        #     self.state_start = [self.current_x, self.current_y, self.current_yaw]
        #     self.state = 5

        # # Start Play on Area3 take ball    
        # elif (self.state == 5 and self.current_x > 8.0 and self.current_x < 12.0 and self.current_y >= -1.6 and self.current_y < 0.1):
        #     self.goalball[0] = self.x*np.cos(self.current_yaw) - self.y*np.sin(self.current_yaw) + self.current_x
        #     self.goalball[1] = self.x*np.sin(self.current_yaw) + self.y*np.cos(self.current_yaw) + self.current_y
        #     self.goalball[2] = self.yaw + self.current_yaw
             
        #     if (self.goalball[2] > 2*np.pi or self.goalball[2] <-2*np.pi):
        #         self.goalball[2]=0.0
        #     if (self.goalball[2] >= np.pi):
        #         self.goalball[2] -=2*np.pi
        #     elif (self.goalball[2] <= -np.pi):
        #         self.goalball[2] +=2*np.pi
            
        #     self.state_start = [self.current_x, self.current_y, self.current_yaw]
        #     self.state_end = self.goalball
        #     self.get_logger().info('Goal : "[%f, %f, %f]"' % (self.goalball[0], self.goalball[1], self.goalball[2]))
        
        # elif (self.state == 6):
        #     self.state_start = [self.current_x, self.current_y, self.current_yaw]
        #     self.state_end = self.silo3
        #     self.get_logger().info('Goal : "[%f, %f, %f]"' % (self.silo3[0], self.silo3[1], self.silo3[2]))
        #     time.sleep(1)
        #     self.state = 7
    
        # elif (self.current_y >(self.silo3[1]-0.1)  and self.current_y < (self.silo3[1]+0.1) and self.state == 7):
        #      time.sleep(2)
        #      self.state_start = [self.current_x, self.current_y, self.current_yaw]
        #      self.state_end = self.goaltake_ball
        #      self.get_logger().info('Goal : "[%f, %f, %f]"' % (self.silo3[0], self.silo3[1], self.silo3[2]))
        #      self.state = 5
        # elif (self.laserX >= 0.3 and self.laserX <= 0.45 and self.state == 7):
        #     time.sleep(2)
        #     self.state_start = [self.current_x, self.current_y, self.current_yaw]
        #     self.state_end = self.goaltake_ball
        #     self.get_logger().info('Goal : "[%f, %f, %f]"' % (self.silo3[0], self.silo3[1], self.silo3[2]))
        #     self.state = 5
        #  print(self.state)
        ### TESTING TO TAKE BALL IN AREA3 #####
         if (self.goal == "goal1" and self.state == 0):
            self.state_end = self.goal0
            self.state_start = [self.current_x, self.current_y, self.current_yaw]
            self.state = 1
         
         elif (self.current_x >=2.8 and self.current_x <=6.0  and self.state == 1):
            curr_timeball = time.time()
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
            self.state_start = [self.current_x, self.current_y, self.current_yaw]
            self.state_end = [self.goalball[0],self.goalball[1],self.goalball[2]]
            self.errorx = self.goalball[0] - self.current_x
            if (abs(self.errorx)<=0.5  and (curr_timeball - self.prev_timeball) > 4 ):
              self.state_end = self.goaltake_ball
              self.state = 1
              self.prev_timeball = curr_time
              self.get_logger().info('No ball')
        #  elif (abs(self.errorx)<0.1 and abs(self.errory)<0.1 and (curr_timeball - self.prev_time) > 4 and self.state == 1):
         elif (abs(self.errorx)<=0.2  and (curr_time - self.prev_time) > 6 ):
              self.state_end = self.goaltake_ball
              self.prev_time = curr_time
              self.state = 1
              self.get_logger().info('No ball')
         elif (self.state == 6):
            self.step = 1
            self.state_start = [self.current_x, self.current_y, self.current_yaw]
            if (self.laserY >= 1.8 and self.step == 1):
                silo = [1,2,3]
                self.silo = np.random.choice(silo)
                if (self.silo == 1):
                    self.state_end = [2.6,1.5,0.0]
                    self.silo_end = self.silo1
                elif (self.silo == 2):
                    self.state_end = [2.6,0.75,0.0]
                    self.silo_end = self.silo2
                elif (self.silo == 3):
                    self.state_end = [2.6,0.0,0.0]
                    self.silo_end = self.silo3
                self.step = 0
            elif (self.laserY < 1.8 and self.step == 1):
                silo = [3,4]
                self.silo= self.Goal = np.random.choice(silo)
                if (self.silo == 3):
                    self.state_end = [2.6,0.0,0.0]
                    self.silo_end = self.silo3
                elif (self.silo == 4):
                    self.state_end = [2.6,-0.75,0.0]
                    self.silo_end = self.silo4
                # elif (self.silo == 5):
                    # self.state_end = [3.6,-1.5,0.0]
                #     self.silo_end = self.silo5
                self.step = 0
            if (self.current_x>=self.state_end[0]-0.1 and self.current_x<=self.state_end[0]+0.1):
                self.state_end = self.silo_end
                self.state = 7
            self.get_logger().info('Goal_Silo :"%d"' %self.silo)
         elif (self.current_x >=(self.state_end[0]-0.2)  and self.current_x <= self.state_end[0]+0.2 and self.state == 7):
            self.state_start = [self.current_x, self.current_y, self.current_yaw]
            if (self.silo == 1):
                self.laser_siloY = 3.2+1.55 # laser_ref + Y_ref
            elif (self.silo == 2):
                self.laser_siloY = 2.52+0.75
            elif (self.silo == 3):
                self.laser_siloY = 1.78+0.0
            elif (self.silo == 4):
                self.laser_siloY = 1.03-0.75
            # elif (self.silo == 5):
            #     self.laser_siloY = 0.26-1.55
            self.state_end = [-self.laserX + 0.25, -self.laserY+self.laser_siloY ,0.0]
            self.get_logger().info('Goal_Silo :"%d"' %self.silo)
            self.state = 8
         elif ((self.laserX >= 0.1 and self.laserX <= 0.35)  and self.state == 8):
            time.sleep(1.0)
            self.state_start = [self.current_x, self.current_y, self.current_yaw]
            self.state_end = self.goaltake_ball
            self.state = 1
         self.get_logger().info('Goal : "[%f, %f, %f]"' % (self.state_end[0], self.state_end[1], self.state_end[2]))
         self.get_logger().info('current : "[%f, %f, %f]"' % (self.current_x, self.current_y, self.current_yaw))
def main(args=None):
    rclpy.init(args=args)
    pid_bf = Buffalo_PID()
    rclpy.spin(pid_bf)
    pid_bf.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()