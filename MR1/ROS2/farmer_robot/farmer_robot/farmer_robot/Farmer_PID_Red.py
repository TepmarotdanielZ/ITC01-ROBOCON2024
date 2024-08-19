import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Float32MultiArray,Float32,Int8,String,UInt16,UInt8MultiArray, UInt8
from sensor_msgs.msg import Imu
import numpy as np
from farmer_robot.pid_controller import PIDController
from farmer_robot.mecanum_farmer import FarmerModel
import math
import time


class Farmer_PID(Node):
    def __init__(self):
        super().__init__('FM_PID')

        ###**** SUBSCRIPTION *****######
        self.mode_sub = self.create_subscription(String, '/mode', self.subscrib_mode, 10)
        self.manual_sub = self.create_subscription(Twist,'/cmd_vel', self.subscrib_manual, 10)
        self.feedback_sub = self.create_subscription(Float32MultiArray, '/external', self.External_feedback, 10)
        self.encoder_sub = self.create_subscription(Float32MultiArray, '/encoder', self.Encoder_feedback, 10)
        self.laser_sub = self.create_subscription(Float32MultiArray, '/laser', self.laser_callback, 10)
        self.position_sub = self.create_subscription(UInt8, '/position', self.position_callback, 10)

        ###**** PUBLISHER ****#####
        self.input_pub = self.create_publisher(Float32MultiArray, '/control', 10)
        self.state_pub = self.create_publisher(UInt8MultiArray, '/state', 10)
        self.position_motor_pub = self.create_publisher(Float32, '/motor_pos', 10)
        ###*** TIMER ****####
        timer = 0.01
        self.pid_timer = self.create_timer(timer,self.Calculate_position)
        self.goal_timer = self.create_timer(timer,self.Calculate_goal)

        ###*** VARIABLE PARAMETER ***###
        ### Parameter of Robot
        self.R = 0.075
        self.Lx = 0.75/2
        self.Ly = 0.75/2 
        self.mecanum=FarmerModel(self.R,self.Lx,self.Ly)
        self.sampling_time = 0.01
        
        ## Variable PID Auto
        self.dt = 0.01
        self.Kp = np.array([1.5, 1.5, 1.0])
        self.Ki = np.array([0.0001, 0.0001, 0.0001])
        self.Kd = np.array([0.004, 0.004, 0.004])
        self.integral_min = -0.4
        self.integral_max = 0.4
        self.output_min = -0.4
        self.output_max = 0.4
        self.integral_min_Angular = -0.5
        self.integral_max_Angular = 0.5
        self.output_min_Angular = -0.5
        self.output_max_Angular = 0.5
        self.alpha = 0.7
        self.pid_X= PIDController(self.Kp[0], self.Ki[0], self.Kd[0], self.dt, self.alpha, self.integral_min, self.integral_max, self.output_min, self.output_max)
        self.pid_Y= PIDController(self.Kp[1], self.Ki[1], self.Kd[1], self.dt, self.alpha, self.integral_min, self.integral_max, self.output_min, self.output_max)
        self.pid_Yaw= PIDController(self.Kp[2], self.Ki[2], self.Kd[2], self.dt, self.alpha, self.integral_min_Angular, self.integral_max_Angular, self.output_min_Angular, self.output_max_Angular)
        self.output_vx = 0.0
        self.output_vy = 0.0
        self.output_omega = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.omega = 0.0
        # MANUAL VARIABLE
        self.Vx = 0.0
        self.Vy = 0.0
        self.Omega = 0.0

        # SET GOAL
        self.sampling_time = 0.01
        self.mode_status = None
        self.goal = None
        self.X_start = 0.0
        self.Y_start = 0.0
        self.Yaw_start = 0.0
        self.Goal = [0.0, -0.0, 0.0]
        self.goal1 = [0.4, 0.0, 0.0]
        self.goal2 = [0.4, -1.38, 0.0]
        self.goal3 = [2.0, -3.5, -1.57] #x_prev = 2.225
        self.goal_end = [0.0, 0.0, 0.0]
        self.laser_refX = [-3.33, -3.38, -3.42, -1.05]  #s4 = -0.98 
        # self.laser_refY = [-2.07, -2.98, -3.88, 1.17, 1.68, 2.07, 2.62, 3.1, 3.60]
        self.laser_refY = [-2.1, -3.0, -3.88, 1.17, 1.7, 2.2, 2.7, 3.2, 3.60]
        # self.motor_pos = [-0.66, 0.48, -0.48, 0.30, -0.30, 0.1]
        self.motor_pos = [-0.66, 0.65, -0.65, 0.46, -0.46, 0.25]
        self.motor_col = 0.0
        # SENSOR FEEDBACK   
        self.current_x = -0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.laserX = 0.0
        self.laserY = 0.0
        self.W1 = 0.0
        self.W2 = 0.0
        self.W3 = 0.0
        self.W4 = 0.0
        self.vx_ex = 0.0
        self.vy_ex = 0.0
        self.yaw_imu = 0.0
        self.Wx = 0.0
        self.Wy = 0.0
        self.state = 0
        self.Position = 0
        self.W = [0.0, 0.0, 0.0, 0.0]

        # CalCulate Error
        self.errorX = [self.goal_end[0] - self.current_x]
        self.errorY = [self.goal_end[1] - self.current_y]
        self.errorYaw = [self.goal_end[2] - self.current_yaw]

    def subscrib_manual(self, V):
        self.Vx = V.linear.x
        self.Vy = V.linear.y
        self.Omega = V.angular.z
        
    def subscrib_mode(self, mode):
        self.mode_status = mode.data

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
    
    def position_callback(self, position):
        self.Position = position.data

    def Calculate_position(self):
        self.Vx_ex = self.Wx*np.cos(self.yaw_imu) - self.Wy*np.sin(self.yaw_imu)
        self.Vy_ex = self.Wx*np.sin(self.yaw_imu) + self.Wy*np.cos(self.yaw_imu)
        self.current_x = self.current_x + self.Vx_ex*self.dt
        self.current_y = self.current_y + self.Vy_ex*self.dt
        self.current_yaw = self.yaw_imu

        self.errorX.append(self.goal_end[0] - self.current_x)
        self.errorY.append(self.goal_end[1] - self.current_y)
        self.errorYaw.append(self.goal_end[2] - self.current_yaw)

        self.output_vx = self.pid_X.calculate_pid(self.errorX)
        self.output_vy = self.pid_Y.calculate_pid(self.errorY)
        self.output_omega = self.pid_Yaw.calculate_pid(self.errorYaw)

        self.vx = self.output_vx*np.cos(self.current_yaw)  + self.output_vy*np.sin(self.current_yaw)
        self.vy = -self.output_vx*np.sin(self.current_yaw) + self.output_vy*np.cos(self.current_yaw)
        self.omega = self.output_omega

        if (self.mode_status == "manual"):
            V = Float32MultiArray()
            W = self.mecanum.inverse_kinematic(self.Vx,self.Vy,self.Omega,"numpy")
            V.data = [W[0],W[1],W[2],W[3]]  
            self.input_pub.publish(V)
            # self.get_logger().info('Velocity_manual : "[%f, %f, %f, %f]"' % (W[0], W[1], W[2], W[3]))
        elif (self.mode_status == "auto"):
            V = Float32MultiArray()
            W = self.mecanum.inverse_kinematic(self.vx,self.vy,self.omega,"numpy")
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
        # self.get_logger().info('Mode : %s' %(self.mode_status))
        self.get_logger().info('Current: X = %f, Y = %f, Yaw = %f' %(self.current_x, self.current_y, self.current_yaw))
    def Calculate_goal(self):
        motor =  Float32()
        State = UInt8MultiArray()
        errorX = self.goal_end[0] - self.current_x
        errorY = self.goal_end[1] - self.current_y
        errorYaw = self.goal_end[2] - self.current_yaw
        if (self.Position == 1 ):
            error_laserX = self.laser_refX[0] + self.laserX
            error_laserY = self.laser_refY[0] + self.laserY
            if (self.state == 0):
                self.goal_end = self.goal1
                self.state = 1
            elif (abs(errorX)<=0.05 and self.state == 1):
                self.goal_end = self.goal2
                self.state = 2
            elif (abs(errorY)<=0.1 and abs(error_laserX)>0.03 and abs(error_laserY)>0.03  and self.state == 2):
                self.goal_end = [error_laserX+self.current_x, error_laserY+self.current_y, 0.0]
            elif (abs(error_laserX)<=0.03 and abs(error_laserY)<=0.03 and self.state == 2):
                self.goal_end = [self.current_x, self.current_y, 0.0]
                self.state = 3

        elif (self.Position == 2 ):
            error_laserX = self.laser_refX[1] + self.laserX
            error_laserY = self.laser_refY[1] + self.laserY
            if (abs(error_laserY)>0.03 and self.state == 3):
                self.goal_end = [0.4, error_laserY+self.current_y, 0.0]
                self.state = 15
            elif (abs(errorX)<=0.05 and self.state ==15):
                self.goal_end = [error_laserX+self.current_x, error_laserY+self.current_y, 0.0] 
                self.state = 16
            elif (abs(error_laserX)<=0.03  and abs(error_laserY)<=0.03 and self.state ==16):
                self.goal_end = [self.current_x, self.current_y, 0.0]
                self.state = 4
        
        elif (self.Position == 3):
            error_laserX = self.laser_refX[2] + self.laserX
            error_laserY = self.laser_refY[2] + self.laserY
            if (abs(error_laserY)>0.03 and self.state == 4):
                self.goal_end = [0.4, error_laserY+self.current_y, 0.0]
                self.state = 17
            elif (abs(errorX)<=0.1  and self.state == 17):
                self.goal_end = [error_laserX+self.current_x, error_laserY+self.current_y, 0.0]
                self.state = 18
            elif (abs(error_laserX)<=0.03 and abs(error_laserY)<=0.03 and self.state == 18):
                self.goal_end = [self.current_x, self.current_y, 0.0]
                self.state = 5

        elif (self.Position == 4):
            error_laserX = self.laser_refX[3] + self.laserY
            error_laserY = self.laser_refY[3] - self.laserX
            if (self.state == 5):
                self.goal_end = self.goal3
                self.state = 6
            elif (abs(errorX)<=0.1 and abs(errorY)<=0.1 and abs(errorYaw)<=0.1 and self.state == 6):
                if (abs(error_laserX)>0.03 and abs(error_laserY)>0.03):
                    self.goal_end = [self.current_x+error_laserX, self.current_y+error_laserY, -1.57]
                elif (abs(error_laserX)<=0.03 and abs(error_laserY)<=0.03):
                    self.goal_end = [self.current_x, self.current_y, -1.57]
                    self.state = 7
        
        elif (self.Position == 5):
            error_laserX = self.laser_refX[3] + self.laserY
            error_laserY = self.laser_refY[4] - self.laserX
            if (abs(error_laserX)>=0.03 and abs(error_laserY)>=0.03 and self.state == 7):
                self.goal_end = [self.current_x+error_laserX, self.current_y+error_laserY, -1.57]
            elif (abs(error_laserX)<0.03 and abs(error_laserY)<0.03 and self.state == 7):
                self.goal_end = [self.current_x, self.current_y, -1.57]
                self.state = 8
        
        elif (self.Position == 6):
            error_laserX = self.laser_refX[3] + self.laserY
            error_laserY = self.laser_refY[5] - self.laserX
            if (abs(error_laserX)>=0.03 and abs(error_laserY)>=0.03 and self.state == 8):
                self.goal_end = [self.current_x+error_laserX, self.current_y+error_laserY, -1.57]
            elif (abs(error_laserX)<0.03 and abs(error_laserY)<0.03 and self.state == 8):
                self.goal_end = [self.current_x, self.current_y, -1.57]
                self.state = 9
        
        elif (self.Position == 7):
            error_laserX = self.laser_refX[3] + self.laserY
            error_laserY = self.laser_refY[6] - self.laserX
            if (abs(error_laserX)>0.03 and abs(error_laserY)>0.03 and self.state == 9):
                self.goal_end = [self.current_x+error_laserX, self.current_y+error_laserY, -1.57]
            elif (abs(error_laserX)<=0.03 and abs(error_laserY)<=0.03 and self.state == 9):
                self.goal_end = [self.current_x, self.current_y, -1.57]
                self.state = 10

        elif (self.Position == 8):
            error_laserX = self.laser_refX[3] + self.laserY
            error_laserY = self.laser_refY[7] - self.laserX
            if (abs(error_laserX)>0.03 and abs(error_laserY)>0.03 and self.state == 10):
                self.goal_end = [self.current_x+error_laserX, self.current_y+error_laserY, -1.57]
            elif (abs(error_laserX)<=0.03 and abs(error_laserY)<=0.03 and self.state == 10):
                self.goal_end = [self.current_x, self.current_y, -1.57]
                self.state = 11
        
        elif (self.Position == 9):
            error_laserX = self.laser_refX[3] + self.laserY
            error_laserY = self.laser_refY[8] - self.laserX
            if (abs(error_laserX)>0.03 and abs(error_laserY)>0.03 and self.state == 11):
                self.goal_end = [self.current_x+error_laserX, self.current_y+error_laserY, -1.57]
            elif (abs(error_laserX)<=0.03 and abs(error_laserY)<=0.03 and self.state == 11):
                self.goal_end = [self.current_x, self.current_y, -1.57]
                self.state = 12
        State.data = [self.state, self.Position]
        self.state_pub.publish(State)
        # elif (self.Position == 10):
        #     error_laserX = self.laser_refX[3] + self.laserY
        #     error_laserY = self.laser_refY[8] - self.laserX
        #     if (abs(error_laserX)>0.03 and abs(error_laserY)>0.03 and self.state == 11):
        #         self.goal_end = [self.current_x+error_laserX, self.current_y+error_laserY, -1.57]
        #     elif (abs(error_laserX)<=0.03 and abs(error_laserY)<=0.03 and self.state == 11):
        #         self.goal_end = [self.current_x, self.current_y, -1.57]
        #         self.state = 12
        self.get_logger().info('Mode : %s, Position: %d, State: %d' %(self.mode_status, self.Position, self.state))
        ### CALCULATE GAOL MOTOR
        if (self.Position == 1 and self.state ==1):
            motor.data = self.motor_pos[0]
            self.motor_col = self.motor_pos[0]
            self.position_motor_pub.publish(motor)
        elif (self.Position == 1 and self.state == 3):
            motor.data = self.motor_pos[1]
            self.motor_col = self.motor_pos[0]
            self.position_motor_pub.publish(motor)
        elif (self.Position == 2 and self.state == 15):
            motor.data = self.motor_pos[2]
            self.motor_col = self.motor_pos[0]
            self.position_motor_pub.publish(motor)
        elif (self.Position == 2 and self.state == 4):
            motor.data = self.motor_pos[3]
            self.motor_col = self.motor_pos[0]
            self.position_motor_pub.publish(motor)
        elif (self.Position == 3 and self.state == 17):
            motor.data = self.motor_pos[4]
            self.motor_col = self.motor_pos[0]
            self.position_motor_pub.publish(motor)
        elif (self.Position == 3 and self.state == 5):
            motor.data = self.motor_pos[5]
            self.motor_col = self.motor_pos[0]
            self.position_motor_pub.publish(motor)
        elif (self.Position>3):
            motor.data = -0.05 #drop
            self.motor_col = -0.05
            self.position_motor_pub.publish(motor)
def main(args=None):
    rclpy.init(args=args)
    pid_fm = Farmer_PID()
    rclpy.spin(pid_fm)
    pid_fm.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()