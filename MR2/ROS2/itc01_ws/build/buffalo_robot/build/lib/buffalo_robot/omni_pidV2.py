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


def plot_arrow(x, y, yaw, length=0.05, width=0.25, fc="r", ec="k"):
        """
        Plot arrow
        """
        if not isinstance(x, float):
            for ix, iy, iyaw in zip(x, y, yaw):
                plot_arrow(ix, iy, iyaw)
        else:
            plt.arrow(x, y, length * math.cos(yaw), length * math.sin(yaw),
                    fc=fc, ec=ec, head_width=width, head_length=width)
            plt.plot(x, y)

class PID_Simulation(Node):
    def __init__(self):
        super().__init__('pid_simu')

        ## Subscription teleop twist keyboard
        ## Subscription velocity to manual
        self.manual_sub = self.create_subscription(Twist,'/manual', self.subscrib_manual, 10)

        ## Subscription Rotary and IMU Feedback
        self.feedback_sub = self.create_subscription(Float32MultiArray, '/external', self.subscrib_feedback, 10)
        self.imu_sub = self.create_subscription(Imu,'/imu',self.subscript_imu,10)
        ## Subscription from Encoder motor
        self.encoder_sub = self.create_subscription(Float32MultiArray, '/encoder', self.subscrib_encoder, 10)

        ## Subscription Mode
        self.mode_sub = self.create_subscription(String, '/mode', self.subscrib_mode, 10)

        self.sub_ball = self.create_subscription(Float32MultiArray, '/ball_distance', self.subscrib_ball, 10)
        self.sub_shoot = self.create_subscription(Int8, "/shoot_already", self.shooter_already, 10)
        self.sub_silo = self.create_subscription(Int8, '/silo_goal', self.ball_already, 10)
        self.laser_sub = self.create_subscription(Float32MultiArray, '/laser', self.laser_callback, 10)
        ## Publish input control manual
        self.input_pub = self.create_publisher(Float32MultiArray, '/control', 10)
        ## Publisher Output PID
        timer = 0.01
        self.output_pub = self.create_publisher(Float32MultiArray,'/pointer',10)
        self.output_timer = self.create_timer(timer,self.Calculate_position)
        self.ball_timer = self.create_timer(0.01,self.calculate_goal)
        
        # ## Publish to can
        # timer = 0.01
        # self.o_pub = self.create_publisher(Float32MultiArray,'/pointer',10)
        # self.output_timer = self.create_timer(timer,self.Calculate_position)
        # timer_transmit = 0.01
        # self.sim_timer = self.create_timer(timer_transmit, self.transmit_velocity)
        ## Variable Manual
        self.Vx = 0.0
        self.Vy = 0.0
        self.Omega = 0.0
        self.sim_time = 100

        ### Parameter of Robot
        self.R = 0.06
        self.Lx = 0.45/2
        self.Ly = 0.45/2 
        self.omni=Omni_model(self.R,self.Lx,self.Ly)
        self.sampling_time = 0.01
        self.mode_status = "auto"
        self.goal = "goal1"
        ## Variable PID
        self.dt = 0.01
        # self.Kp = np.array([2.0, 2.0, 1.2])
        # self.Ki = np.array([0.0000, 0.0000, 0.0000])
        # self.Kd = np.array([0.008, 0.008, 0.004])
        self.Kp = np.array([1.5, 1.0, 1.2])
        self.Ki = np.array([0.0000, 0.0000, 0.0000])
        self.Kd = np.array([0.008, 0.008, 0.004])
        self.integral_min = -1.0
        self.integral_max = 1.0
        self.output_min = -1.0
        self.output_max = 1.0
        self.integral_min_Angular = -1.0
        self.integral_max_Angular = 1.0
        self.output_min_Angular = -1.0
        self.output_max_Angular = 1.0
        self.alpha = 0.7
        # self.integral_min = -1.4
        # self.integral_max = 1.4
        # self.output_min = -1.4
        # self.output_max = 1.4
        # self.integral_min_Angular = -1.0
        # self.integral_max_Angular = 1.0
        # self.output_min_Angular = -1.0
        # self.output_max_Angular = 1.0
        self.pid_X= PIDController(self.Kp[0], self.Ki[0], self.Kd[0], self.dt, self.alpha, self.integral_min, self.integral_max, self.output_min, self.output_max)
        self.pid_Y= PIDController(self.Kp[1], self.Ki[1], self.Kd[1], self.dt, self.alpha, self.integral_min, self.integral_max, self.output_min, self.output_max)
        self.pid_Yaw= PIDController(self.Kp[2], self.Ki[2], self.Kd[2], self.dt, self.alpha, self.integral_min_Angular, self.integral_max_Angular, self.output_min_Angular, self.output_max_Angular)
        self.output_vx = 0.0
        self.output_vy = 0.0
        self.output_omega = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.omega = 0.0

        ## Variable Auto
        self.vx = 0.0
        self.vy = 0.0
        self.omega = 0.0
        self.X_start = 0.0
        self.Y_start = 0.0
        self.Yaw_start = 0.0
        self.Goal = np.array([0.0, 0.0, 0.0])
        # self.goal0 = np.array([6.0, 0.0, 0.0])
        self.goal1 = np.array([6.3, -3.8, 0.0])
        self.goal2 = np.array([9.5, -3.8, 0.0])
        self.goal3 = np.array([9.5, -3.8, 1.57])
        self.goal4 = np.array([10.5, -1.5, 1.57])
        self.goalball = np.array([0.0,0.0,0.0])
        # self.silo1 = np.array([8.1, -4.8, 1.57])
        # self.silo2 = np.array([8.9, -4.8, 1.57])
        # self.silo3 = np.array([10.2, -5.5, 1.57])
        # self.silo4 = np.array([10.4, -4.8, 1.57])
        # self.silo5 = np.array([11.15, -4.8, 1.57])
        # self.goaltake_ball = np.array([10.5, -2.0, 1.57])
        # Testing take ball
        self.silo = 0
        self.goal0 = np.array([2.5, -0.0, 0.0])
        self.silo1 = np.array([0.0, 1.6, 0.0])
        self.silo2 = np.array([0.0, 0.8,0.0])
        self.silo3 = np.array([-0.0,0.0,0.0])
        self.silo4 = np.array([0.0,-0.8,0.0])
        self.silo5 = np.array([0.0,-1.6,0.0])
        self.silo_end = np.array([0.0, 0.0, 0.0])
        self.goaltake_ball = np.array([2.5, -0.0, 0.0])
        self.ref_path = [0.0, 0.0 ,0.0]
        self.state_end = np.array([0.0, 0.0, 0.0])
        self.state_start = np.array([0.0 , 0.0, 0.0])
        self.current_x = -0.00
        self.current_y = 0.00
        self.current_yaw = 0.0
        self.measurement_x = 0.0
        self.measurement_y = 0.0
        self.measurement_yaw = 0.0
        self.errorX = [self.ref_path[0] - self.current_x]
        self.errorY = [self.ref_path[1] - self.current_y]
        self.errorYaw = [self.ref_path[2] - self.current_yaw]
        self.hist_x = [self.current_x]
        self.hist_y = [self.current_y]
        self.hist_yaw = [self.current_yaw]
        self.state = 0
        # self.goal = None
        
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
        self.ball_ok = 0
        self.distance = 0.0
        self.silo_goal = 0
        self.laserX = 0.0
        self.lasery = 0.0
        self.laser_siloY = 0.0
        self.laser_siloX = 0.0
        self.step = 0
   
    def laser_callback(self, laser):
        self.laserX = laser.data[0]
        self.laserY = laser.data[1]
    def ball_already(self, silo):
        self.silo_goal = silo.data
        # self.get_logger().info('Goal : "[%d]"' % (self.silo_goal))
        
    def shooter_already(self, ball):
        self.ball_ok = ball.data

    def subscrib_manual(self, V):
        self.Vx = V.linear.x
        self.Vy = V.linear.y
        self.Omega = V.angular.z
    def subscrib_mode(self, mode):
        self.mode_status = mode.data
    
    def subscrib_ball(self, ball):
        self.distance = ball.data[0] 
        yaw = -1*ball.data[1]*np.pi/180 + 0.01
        self.x = self.distance*np.cos(self.yaw)
        self.y = self.distance*np.sin(self.yaw)
        self.yaw = yaw 
        # self.state = 5
        if (self.distance <= 0.35 and self.distance >= 0.2):
            time.sleep(0.5)
            self.state = 6
        
    def subscrib_feedback(self, current):
        self.current_x = current.data[0]
        self.current_y = current.data[1]
        self.current_yaw = current.data[2]
        
    def subscrib_encoder(self, W):
        self.W1 = W.data[0]
        self.W2 = W.data[1]
        self.W3 = W.data[2]
        self.W4 = W.data[3]
        
    def subscript_imu(self,imu):
        q1 = imu.orientation.x
        q2 = imu.orientation.y
        q3 = imu.orientation.z
        q4 = imu.orientation.w
        siny_cosp = 2 * (q4*q3 + q1*q2)
        cosy_cosp = 1 - 2 * (q2*q2 + q3*q3)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

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
        # erroryaw = self.goalball[2] - self.current_yaw
        # if (abs(erroryaw) >= 0.5 and self.state ==1):
        #     self.vx = 0.0
        #     self.vy = 0.0
        # elif (abs(erroryaw)<0.3 and self.state == 1):
        #     self.vx = self.vx
        #     self.vy = self.vy
        if (self.mode_status == "manual"):
            V = Float32MultiArray()
            W = self.omni.inverse_kinematic(self.Vx,self.Vy,self.Omega,"numpy")
            V.data = [W[0],W[1],W[2],W[3]]  
            self.input_pub.publish(V)
            # self.get_logger().info('Velocity control : "[%f, %f, %f, %f]"' % (W[0], W[1], W[2], W[3]))
        elif (self.mode_status == "auto"):
            V = Float32MultiArray()
            W = self.omni.inverse_kinematic(self.vx,self.vy,self.omega,"numpy")
            if (W[0] < 3.0 and W[0] > - 3.0):
                W[0] = 0.0
            if (W[1] < 3.0 and W[1] > - 3.0):
                W[1] = 0.0
            if (W[2] < 3.0 and W[2] > - 3.0):
                W[2] = 0.0
            if (W[3] < 3.0 and W[3] > - 3.0):
                W[3] = 0.0
            V.data = [W[0],W[1],W[2],W[3]] 
            self.input_pub.publish(V)

            # self.get_logger().info('Goal : "[%f, %f, %f]"' % (self.output_vx, self.output_vy, self.output_omega))
    def calculate_goal(self):
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
            self.get_logger().info('Goal : "[%f, %f, %f]"' % (self.goal0[0], self.goal0[1], self.goal0[2]))
            self.state = 1
         
         elif (self.current_x >=2.2  and self.state == 1):
            self.goalball[0] = self.x*np.cos(self.current_yaw) - self.y*np.sin(self.current_yaw) + self.current_x
            self.goalball[1] = self.x*np.sin(self.current_yaw) + self.y*np.cos(self.current_yaw) + self.current_y
            self.goalball[2] = self.yaw + self.current_yaw
             
            if (self.goalball[2] > 2*np.pi or self.goalball[2] <-2*np.pi):
                self.goalball[2]=0.0
            if (self.goalball[2] >= np.pi):
                self.goalball[2] -=2*np.pi
            elif (self.goalball[2] <= -np.pi):
                self.goalball[2] +=2*np.pi
            
            self.state_start = [self.current_x, self.current_y, self.current_yaw]
            self.state_end = [self.goalball[0],self.goalball[1],self.goalball[2]]
            # erroryaw = self.goalball[2] - self.current_yaw
            # if (abs(erroryaw) >= 0.3):
            
            # if (self.current_yaw>= self.goalball[2]-0.3 and self.current_yaw <= self.goalball[2]+0.3):
            #     self.state_end = [self.goalball[0],self.goalball[1],self.goalball[2]]
            self.step = 1
            # if (self.current_x >= (self.goalball[0]-0.1) and self.current_x <= self.current_x <= (self.goalball[0]+0.1)):
            #     time.sleep(1)
            #     self.goaltake_ball
            self.get_logger().info('Goal : "[%f, %f, %f]"' % (self.goalball[0], self.goalball[1], self.goalball[2]))
         
         elif (self.state == 6 and self.step ==1):
            self.state_start = [self.current_x, self.current_y, self.current_yaw]
            if (self.laserY >= 1.8):
                silo = [1,2,3]
                self.silo = np.random.choice(silo)
                if (self.silo == 1):
                    self.silo_end = self.silo1
                elif (self.silo == 2):
                    self.silo_end = self.silo2
                elif (self.silo == 3):
                    self.silo_end = self.silo3
            elif (self.laserY < 1.8):
                silo = [3,4,5]
                self.silo= self.Goal = np.random.choice(silo)
                if (self.silo == 3):
                    self.silo_end = self.silo3
                elif (self.silo == 4):
                    self.sailo_end = self.silo4
                elif (self.silo == 5):
                    self.silo_end = self.silo5
            self.state_end = self.silo_end
            self.get_logger().info('Goal : "[%f, %f, %f]"' % (self.state_end[0], self.state_end[1], self.state_end[2]))
            self.state = 7
         elif (self.current_x >=(self.silo_end[0]-0.1)  and self.current_x <= self.silo_end[0]+0.8 and self.state == 7):
            self.state_start = [self.current_x, self.current_y, self.current_yaw]
            if (self.silo == 1):
                self.laser_siloY = 3.3+1.8
            elif (self.silo == 2):
                self.laser_siloY = 2.55+0.8
            elif (self.silo == 3):
                self.laser_siloY = 1.8
            elif (self.silo == 4):
                self.laser_siloY = 1.05-0.8
            elif (self.silo == 5):
                self.laser_siloY = 0.3-1.6
            self.state_end = [-self.laserX + 0.25, -self.laserY+self.laser_siloY ,0.0]
            self.state = 8
            self.get_logger().info('Goal : "[%f, %f, %f]"' % (self.state_end[0], self.state_end[1], self.state_end[2]))
         elif ((self.laserX >= 0.1 and self.laserX <= 0.35) and (self.laserY>= (self.state_end[1]+self.laser_siloY-0.3) and self.laserY <= (self.state_end[1]+self.laser_siloY+0.3)) and self.state == 8):
            time.sleep(1)
            self.state_start = [self.current_x, self.current_y, self.current_yaw]
            self.state_end = self.goaltake_ball
            self.get_logger().info('Goal : "[%f, %f, %f]"' % (self.state_end[0], self.state_end[1], self.state_end[2]))
            self.state = 1
        

        
def main(args=None):
    rclpy.init(args=args)
    pid_sim = PID_Simulation()
    rclpy.spin(pid_sim)
    pid_sim.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()