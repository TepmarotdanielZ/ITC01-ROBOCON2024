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
        self.feedback_sub = self.create_subscription(Float32MultiArray, '/external', self.External_feedback, 10)
        self.encoder_sub = self.create_subscription(Float32MultiArray, '/encoder', self.Encoder_feedback, 10)
        self.laser_sub = self.create_subscription(Float32MultiArray, '/laser', self.laser_callback, 10)
        self.sub_ball = self.create_subscription(Float32MultiArray, '/ball_distance', self.ball_callback, 10)
        self.sub_typeball = self.create_subscription(String, '/ball_class', self.balltype_callback, 10)
        self.ball_aready_sub = self.create_subscription(UInt8MultiArray, '/ball_already',self.ball_already_callback, 10)
       
        # self.sub_imu = self.create_subscription(Vector3, "/imu/data", self.Imu_callback, 20)
        ###**** PUBLISHER ****#####
        self.mode_pub = self.create_publisher(String, '/mode', 10)
        self.manual_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.state_pub = self.create_publisher(UInt16, '/state', 10)
        self.motor_pub = self.create_publisher(UInt8, '/motor', 10)
        self.goal_pub = self.create_publisher(Float32MultiArray, '/goal_end', 10)
        ###*** TIMER ****####
        timer = 0.01
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
        self.Goal = [0.0, 0.0, 0.0]
        self.goal0 = [6.0, -0.1, 0.0]
        self.goal1 = [6.0, -3.5, 0.0]
        self.goal2 = [10.0, -3.5, 0.0]
        self.goal3 = [10.0, -1.2, 1.57]
        self.goal4 = [10.0, -1.2, 1.57]
        self.goalball = [0.0,0.0,0.0]
       
        self.goal_limitball = [0.0, 0.0]
        self.goaltake_ball = [9.65, -2.0, 1.57]
        # When Retry
        self.goal0_retry = [0.8, -1.2, 0.0]
        self.goal1_retry = [0.8, -3.3, 0.0]
        self.goal2_retry = [4.475, -3.3, 0.0]
        self.goal3_retry = [4.475, -1.5, 1.57]
        self.goal4_retry = [4.475, -1.5, 1.57]
        # Testing take ball
        self.silo = 0
        self.silo_end = [0.0, 0.0, 0.0]
        self.state_end = [0.0, 0.0, 0.0]
        self.state_start = [0.0 , 0.0, 0.0]
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
        self.total_ball = 0
        self.ball_red = 0
        self.ball_blue = 0
        self.distance2 = 0.0
        
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
        self.button = 0
        self.check_three_ball = 0
        self.motor_move = 0
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
        self.data_ball_store = [[0 for _ in range(5)] for _ in range(3)]
        self.robot_move_silo = [0, 0, 0, 0, 0]
        self.silo_rnd1 = [1,2,3]
        self.silo_rnd2 = [3,4]
        self.silo_rnd = [1,2,3,4]
        # PARAMETER LASER
        self.laser_bridgeY = 1.35
        self.laser_refxsilo = -0.01
        self.laser_refysilo = [0.0, 2.9, 2.20, 1.54, 0.89, 0.22]
        self.laser_refywall = [0.0, 2.58, 1.99, 1.26, 0.6]
        self.laser_refxwall = [1.65, 1.98]
        self.laser_refxcheckball = -0.04
        self.laserX = 0.0
        self.lasery = 0.0
        self.laser_siloY = 0.0
        self.laser_siloX = 0.0
        self.laser_refy = 0.0
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
        self.get_logger().info('red_distance : %f ' % (self.distance))
    
    def balltype_callback(self, type):
        self.typeball = type.data
    
    def calculate_goal(self):
        curr_timeball = time.time()
        curr_timeBall = time.time()
        mode = String()
        manual = Twist()
        goal = Float32MultiArray()
        state_msg = UInt16()
        
        self.errorx = self.state_end[0] - self.current_x
        self.errory = self.state_end[1] - self.current_y
        self.erroryaw = self.state_end[2] - self.current_yaw
        if (self.ball_aready == 0 and (self.state == 6 or self.state == 7)):
            self.state = 5
            self.state_end = self.goaltake_ball
            self.mode_status = "auto"
            mode.data = self.mode_status
            self.mode_pub.publish(mode)
            
        if (abs(self.errorx)<=0.04 and abs(self.errory)<=0.05 and self.check_three_ball == 0 and (self.state == 11) ):
            self.state = 12
        elif (abs(self.errorx)<=0.04 and abs(self.errory)<=0.05 and self.check_three_ball ==1 and (self.state == 11 ) ):
            self.silo -=1
            if (self.silo == 0 ):
                self.silo = 5
            self.state = 22
        ##** Auto from Area1 to Area2 : FULL OPTION
        if (self.goal == "success" ):
            self.goal_limitball = [7.5, 13.5] # [min, max]
            if (self.state == 0):
                self.state_end = self.goal0
                self.state = 1
            elif (abs(self.errorx)<=0.1 and self.state == 1):
                self.state_end = self.goal1
                self.state = 2
            elif (abs(self.errory)<=0.1 and self.state == 2):
                error_lasery = -self.laserY+self.laser_bridgeY
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
                error_lasery = -self.laserY+self.laser_bridgeY
                if (abs(error_lasery)>=0.15):
                    self.state_end = [self.current_x,error_lasery+self.current_y,0.0]
                elif (abs(error_lasery)<0.15):
                    self.state_end = [self.goal2_retry[0],self.current_y,self.goal2_retry[2]]
                    self.state = 3
            elif (abs(self.errorx)<=0.1 and self.state == 3):
                self.state_end = self.goal3_retry
                self.state = 4
            elif (abs(self.errory)<=0.1 and self.state == 4):
                self.state_end = self.goal4_retry
                self.state = 5
                self.goal = None
                
        ###*** START PLAY Ball IN AREA3  ***###
        # WHEN ROBOT CATCH BALL
        elif ((self.current_x >= self.goal_limitball[0] and self.current_x < self.goal_limitball[1])  and (self.current_y >= -4.7 and self.current_y < 0.5) and self.state == 5  and self.typeball == "red ball"):
            self.mode_status = "auto"
            mode.data = self.mode_status
            self.mode_pub.publish(mode)
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
            # self.state = 5
        elif ((self.distance<=0.40 and self.distance>0.31) and (curr_timeBall-self.prev_time)>2.5 and self.state == 5 and self.mode_status == "auto"):
            self.mode_status = "manual"
            mode.data = self.mode_status
            self.mode_pub.publish(mode)
            self.Omega = -1.5
            manual.angular.z = self.Omega
            self.manual_pub.publish(manual)
            self.state = 20
            self.prev_time = curr_timeBall
        elif (self.state == 20 and (curr_timeBall-self.prev_time)>1.0):
            self.mode_status = "auto"
            self.state = 5
            self.prev_time = curr_timeBall
        #When NOT SEE BALL
        
        elif ((self.current_y >= -4.5)  and (curr_timeball - self.prev_timeball)>2.5 and self.state == 5  and  (self.typeball == "no_ball" ) and self.state_yaw ==0 ):
            self.mode_status = "manual"
            mode.data = self.mode_status
            self.mode_pub.publish(mode)
            self.Omega = -1.2
            manual.angular.z = self.Omega
            self.manual_pub.publish(manual)
            self.state_yaw = 1
            self.state_count = 0
            self.prev_timeball = curr_timeball 
        elif ((self.current_yaw>=-1.2 and self.current_yaw <=-0.7) and self.mode_status == "manual" and  (self.typeball == "no_ball" ) and self.state == 5 and (curr_timeball - self.prev_timeball)>0.5 and self.state_yaw == 1):  
            self.mode_status = "manual"
            mode.data = self.mode_status
            self.mode_pub.publish(mode)
            self.Omega = 1.2
            manual.angular.z = self.Omega
            self.manual_pub.publish(manual)
            self.prev_timeball = curr_timeball
        elif ((self.current_yaw>= -3.14 and self.current_yaw <= -2.7) and self.mode_status == "manual" and  (self.typeball == "no_ball" ) and self.state == 5 and (curr_timeball - self.prev_timeball)>0.5 and self.state_yaw == 1): 
            self.mode_status = "manual"
            self.Omega = -1.2
            mode.data = self.mode_status
            self.mode_pub.publish(mode)
            manual.angular.z = self.Omega
            self.manual_pub.publish(manual)
            self.state_count +=1
            self.prev_timeball = curr_timeball
        elif (self.state_count == 1 and self.state == 5 and (self.typeball == "no_ball" ) and self.state_move ==0 and (curr_timeball - self.prev_timeball)>0.5):
            self.Omega = 0.0
            self.mode_status = "auto"
            mode.data = self.mode_status
            self.mode_pub.publish(mode)
            self.state_end = [self.current_x+0.3, self.current_y+0.5, 1.57]
            self.state_move = 1
            self.state_yaw = 0
            self.state_count = 0
            self.prev_timeball = curr_timeball
        elif (self.state_count == 1 and self.state == 5 and (self.typeball == "no_ball" ) and self.state_move == 1):
            self.Omega = 0.0
            self.mode_status = "auto"
            mode.data = self.mode_status
            self.mode_pub.publish(mode)
            self.state_end = [self.current_x-1.0, self.current_y+0.5, 1.57]
            self.state_count = 0
            self.state_move = 0
            self.state_yaw = 0
         
        # WHEN ROBOT TAKE BALL AREADY
        elif (self.state == 6 and self.mode_status == "auto"):
            if (self.current_x >= self.goal_limitball[0] and self.current_x <= self.goal_limitball[1] -3.0):
                self.state_end = [self.current_x+0.5, self.current_y-2.5, 0.75]
            elif ((self.laserY >=0.02 and self.laserY<=0.9 and self.current_x>=self.goal_limitball[1] -3.5) or (self.current_x>=self.goal_limitball[1]-2.0 and self.current_x<=self.goal_limitball[1])):
                self.state_end = [self.current_x-1.0, self.current_y-2.5, 0.75]
            else:
                self.state_end = [self.current_x-0.5, self.current_y-2.5, 0.75]
            self.state = 7
            self.state_yaw = 0
        elif (abs(self.errory)<=0.3 and abs(self.erroryaw)<=0.1 and self.state == 7):
            self.state_end = [self.current_x, self.current_y, 1.57]
            self.state = 17
        # WHEN ROBOT CHECK LASER TO MOVE SILO POSITION_X IN ROBOT NEAR
        elif (abs(self.erroryaw)<=0.1 and abs(self.errory)<=0.1 and not((self.laserX>=self.laser_refxwall[0] and self.laserX<=self.laser_refxwall[0])) and self.state == 17):
            if (self.laserY >= self.laser_refysilo[3]):
                # silo = [1,2,3]
                self.silo = np.random.choice(self.silo_rnd1)
            elif (self.laserY <= self.laser_refysilo[3]):
                # silo = [3,4]
                self.silo = np.random.choice(self.silo_rnd2)
            self.laser_refy = self.laser_refysilo[self.silo]
            self.state = 8
        elif (abs(self.erroryaw)<=0.1 and abs(self.errory)<=0.1 and (self.laserX>=self.laser_refxwall[0] and self.laserX<=self.laser_refxwall[1]) and self.state == 17):
            if (self.laserY >= self.laser_refywall[3]):
                # silo = [1,2,3]
                self.silo = np.random.choice(self.silo_rnd1)
            elif (self.laserY <= self.laser_refywall[3]):
                # silo = [3,4]
                self.silo = np.random.choice(self.silo_rnd2)
            self.state = 8
            self.laser_refy = self.laser_refywall[self.silo]
        
        # WHEN ROBOT START MOVE TO SILO
        elif (self.state == 8 ):
            error_lasery = self.laserY - self.laser_refy
            if (abs(error_lasery)>= 0.2):
                self.state_end = [self.current_x+error_lasery, self.current_y, 1.57]
            elif (abs(error_lasery)< 0.2):
                # self.state_end = [self.current_x, self.current_y, 1.57]
                self.state = 9
            self.get_logger().info('Goal_Silo :"%d"' %self.silo)
            
        elif ((self.state == 9 and (not(self.laserX >= 0.01 and self.laserX <= 1.4)))):
            self.state_end = [self.current_x, self.current_y-1.0, 1.57]
            self.state == 10
       
        #  CHECK  Threee Ball
        elif ((self.laserX >= 0.001 and self.laserX <= 1.4) and (self.state == 10 or self.state==9)):
            error_laserx_py = -self.laserX - self.laser_refxcheckball
            error_lasery_px = self.laserY - self.laser_refysilo[self.silo]
            if (abs(error_lasery_px)>=0.04  and abs(error_laserx_py)>=0.04):
                self.state_end = [self.current_x+error_lasery_px, error_laserx_py+self.current_y, 1.57]
            elif (abs(error_lasery_px)<0.04  and abs(error_laserx_py)<0.04):
                self.state = 11
                time.sleep(1)
        elif ((self.state == 22 )):
            error_laserx_py = -self.laserX - self.laser_refxcheckball
            error_lasery_px = self.laserY - self.laser_refysilo[self.silo]
            if (abs(error_lasery_px)>=0.04 ):
                self.state_end = [self.current_x+error_lasery_px, error_laserx_py+self.current_y, 1.57]
            elif (abs(error_lasery_px)<0.04  and abs(error_laserx_py)<0.04):
                self.state = 11
                time.sleep(1.0)
           
        # WHEN ROBOT CHECK POSITION_Y AND SHOOT BALL TO SILO
        elif ((self.laserX >= 0.01 and self.laserX <= 2.2)  and (self.state == 12)):
            error_laserx_py = -self.laserX - self.laser_refxsilo
            error_lasery_px = self.laserY - self.laser_refysilo[self.silo]
            self.get_logger().info('erorrlaserx: %f, errorrlaeserY: %f' % (error_laserx_py, error_lasery_px))
            if ((abs(error_laserx_py)>=0.02 and (abs(error_lasery_px)>=0.04)) and self.state == 12):
                self.state_end = [self.current_x+error_lasery_px, error_laserx_py+self.current_y, 1.57]
            elif ((abs(error_laserx_py)<0.02 and (abs(error_lasery_px)< 0.04)) and (self.state == 12)):
                self.state = 13
                state_msg.data = self.state
                self.state_pub.publish(state_msg)
                time.sleep(1.7)
                errorsilo3 = self.laserY - self.laser_refysilo[3]
                self.goaltake_ball = [self.current_x+errorsilo3,-2.2, 1.57]
                self.state_end = self.goaltake_ball
                self.state = 14

        # WHEN ROBOT RETURN FROM SILO TO CATCH BALL AGAIN
        elif ((self.laserX<=2.1 and self.laserX>=1.7) and self.state == 14 ):
                errorsilo3 = self.laserY - self.laser_refysilo[3]
                self.state_end = [self.current_x+errorsilo3, self.current_y+2.5, 1.57]
                self.state = 15
        elif (abs(self.errory)<=0.5 and self.state == 15):
                self.state = 5
                self.take_ball = 0
                self.state_move = 0
                self.state_yaw = 0
        mode.data = self.mode_status
        self.mode_pub.publish(mode)    
        goal.data = [self.state_end[0], self.state_end[1], self.state_end[2]]
        self.goal_pub.publish(goal)
        state_msg.data = self.state
        self.state_pub.publish(state_msg)
        self.get_logger().info('Goal : "[%f, %f, %f]"' % (self.state_end[0], self.state_end[1], self.state_end[2]))
        self.get_logger().info('current : "[%f, %f, %f]"' % (self.current_x, self.current_y, self.current_yaw))
        self.get_logger().info('mode : %s ' % (self.mode_status))
        self.get_logger().info('Goal_Silo :"%d", State: %d, goal_mode: %s, check_ball: %d' %(self.silo, self.state, self.typeball, self.check_three_ball))
def main(args=None):
    rclpy.init(args=args)
    pid_bf = Buffalo_PID()
    rclpy.spin(pid_bf)
    pid_bf.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()