## By: YIN CHHEANYUN ##
## Date: 4 jan 2023 ##

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist , Vector3
from std_msgs.msg import Float32MultiArray,Float32, String, Int8
from sensor_msgs.msg import Imu
import can
import numpy as np
from sensor_msgs.msg import Joy
import time
from buffalo_robot.EKF import Extended_kalmanFilter
from numpy.random import randn
from buffalo_robot.omni_buffalo import Omni_model

##*First Subscripber Twist/cmd_vel and Second Publish msg CANBUS to STM32 
##*And Third Publish Data Float32MultiArray/velocity 
def map(Input, min_input, max_input, min_output, max_output):
    value = ((Input - min_input)*(max_output-min_output)/(max_input - min_input) + min_output)
    return value

class ros_node(Node):
    def __init__(self):
        super().__init__('can_node')
        #Publisher Float32MultiArray/velocity
        self.pub_timer = 0.01
        self.encoder_pub = self.create_publisher(Float32MultiArray, '/encoder',10)
        self.external_pub = self.create_publisher(Float32MultiArray, '/external',10)
        self.shoot_pub = self.create_publisher(Int8, "/shoot_already" ,10)
        self.silo_pub = self.create_publisher(Int8, '/silo_goal' , 10)
        self.laser_pub = self.create_publisher(Float32MultiArray, '/laser', 10)
        self.back_timer = self.create_timer(self.pub_timer, self.back_callback)

        #Subscriber Twist/cmd_vel
        self.Input_ = self.create_subscription(Float32MultiArray, '/control', self.sub_control, 10)
        self.Input_ 
        # self.imu = self.create_subscription(Vector3, "/imu/data", self.imu_feedback, 10)
        # self.imu_subscriber = self.create_subscription(Imu, 'imu/data2', self.imu_callback, 10)
        self.collect = self.create_subscription(Float32MultiArray, '/ball_distance', self.collect_control, 10)
        self.shooter = self.create_subscription(String, "/shoot_silo", self.webcame, 10)
        
        #Publish CANBUS to STM3
        self.timer = 0.001
        self.bus =can.interface.Bus(channel='can0', interface ='socketcan', bitrate=1000000)
        self.can_timer_ = self.create_timer(self.timer, self.timerCanCB)

        #Subscription from Joy or cmd_vel to CAN_Tx
        
        self.Omega = 0.0
        self.Speed = 0.0
        self.SpeedAngle = 0.0
        self.i=0
        self.TxData = [128,0,128,0,128,0,128,0]
        self.TxData1 = [0]
        self.TxData2 = [0]
        # self.V_back=[0,0,0,0]
        self.V1_input=0.0
        self.V2_input=0.0
        self.V3_input=0.0
        self.V4_input=0.0
        ###** Recieve from CAN_Rx to Publish check Position
        
        self.first_init = True
        self.prev_tick = np.array([0,0])
        self.curr_tick = np.array([0,0])
        self.rotary = np.zeros(2)
        self.V_Back=[0,0,0,0]
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.W1 = 0.0
        self.W2 = 0.0
        self.V1=0
        self.V2=0
        self.V3=0
        self.V4=0
        self.V1_back=0
        self.V2_back=0
        self.V1_encoder=0.0
        self.V2_encoder=0.0
        self.V3_encoder = 0.0
        self.V4_encoder = 0.0
        self.Omega_back = 0.0
        self.Vx_ex = 0.0
        self.Vy_ex = 0.0
        self.X= 0.0
        self.Y = 0.0
        self.Yaw = 0.0
        self.dt = 0.01 # 10ms
        self.rotary_old = 0
        self.x = 0.0
        
        #Setup Robot
        R = 0.06
        Lx = 0.45/2
        Ly = 0.45/2
        self.omni = Omni_model(R,Lx,Ly)
        self.states = np.array([[0.0],[0.0],[0.0]])

        #Setup EKF
        gain_Q = np.diag([1, 1, 2])
        gain_R = np.diag([1, 1, 1])
        self.P = np.diag([1000, 1000, 1000])
        A = np.diag([1,1,1]) 
        self.EKF = Extended_kalmanFilter(gain_Q,gain_R,A)
        self.process_noise = 0.00*randn(3,1)
        self.measurement_noise = 0.0*randn(3,1)
        self.X_est = 0.0
        self.Y_est = 0.0
        self.Yaw_est = 0.0
        self.state_est = np.array([[self.X_est],[self.Y_est],[self.Yaw_est]])
        self.X_en =  0.0
        self.Y_en = 0.0
        self.Yaw_en = 0.0
        self.phi = 0.0
        self.u1 = 0.0
        self.u2 = 0.0
        self.u3 = 0.0
        self.u4 = 0.0
        self.Vx = 0.0
        self.Vy = 0.0
        self.Omega = 0.0
        self.LaserX = 0
        self.LaserY = 0
        self.laserX = 0.0
        self.laserY = 0.0
        
        # Cammera 
        self.distance = 0
        self.collect_ball = 0
        self.shooter_ball = 0
        self.d = 0.0
        self.goal_silo = 0
        
    
    ###*** Subscriber Twist msg ***###
    def sub_control(self,vin):
        self.V1_input = int(map(vin.data[0], -60.0, 60.0, 0, 65535))
        self.V2_input = int(map(vin.data[1], -60.0, 60.0, 0, 65535))
        self.V3_input = int(map(vin.data[2], -60.0, 60.0, 0, 65535))
        self.V4_input = int(map(vin.data[3], -60.0, 60.0, 0, 65535))
        
        self.TxData[0] = ((self.V1_input & 0xFF00) >> 8)
        self.TxData[1] = (self.V1_input & 0x00FF)
        self.TxData[2] = ((self.V2_input & 0xFF00) >> 8)
        self.TxData[3] = (self.V2_input & 0x00FF)
        self.TxData[4] = ((self.V3_input & 0xFF00) >> 8)
        self.TxData[5] = (self.V3_input & 0x00FF)
        self.TxData[6] = ((self.V4_input & 0xFF00) >> 8)
        self.TxData[7] = (self.V4_input & 0x00FF)
    def collect_control(self,d):
        self.d = d.data[0] 
        self.distance = int(map(d.data[0], 0.0, 20.0, 0, 255))
        self.TxData1[0] = self.distance
        self.collect_msg = can.Message(arbitration_id=0x111, data=self.TxData1, dlc=1, is_extended_id=False)
        self.collect_ball = 1
        self.get_logger().info('Ball distance:[%f]'%(d.data[0]))
    def webcame(self,silo):
        if (silo.data == "No"):
            self.TxData2[0] = 1
        elif (silo.data == "Yes"):
            self.TxData2[0] = 2
        # self.silo_msg = can.Message(arbitration_id=0x222, data=self.TxData2, dlc=1, is_extended_id=False)
        # self.shooter_ball = 1
    # def imu_callback(self,imu_msg):
    #     q1 = imu_msg.orientation.x
    #     q2 = imu_msg.orientation.y
    #     q3 = imu_msg.orientation.z
    #     q4 = imu_msg.orientation.w
    #     siny_cosp = 2 * (q4*q3 + q1*q2)
    #     cosy_cosp = 1 - 2 * (q2*q2+q3*q3*q3)

    #     self.yaw = np.arctan2(siny_cosp, cosy_cosp)
    #     # if self.yaw < -np.pi:
    #     #     self.yaw = self.yaw + 2 * np.pi
    # def imu_feedback(self, axis):
    #     self.roll = axis.x
    #     self.pitch = axis.y
        # self.yaw = axis.z   
        # if (self.yaw > np.pi):
        #         self.yaw = -np.pi

    ###*** Publisher CAN_BUS to STM32 ***### 
    def timerCanCB(self):
        # ball = Int8()
        msg = can.Message(arbitration_id=0x103, data=self.TxData, dlc=8, is_extended_id=False)
        # silo_msg = can.Message(arbitration_id=0x222, data=self.TxData2, dlc=1, is_extended_id=False)
        
        try :
            if (self.collect_ball == 1):
                self.collect_ball= 0
                self.bus.send(self.collect_msg,0.01)
        #     if (self.shooter_ball == 1):
        #         self.shooter_ball = 0
        #         self.bus.send(self.silo_msg,0.01)
        #     self.bus.send(msg,0.01)
        #     # finish_recv = True
        except can.CanError :
            self.get_logger().error('message not send')
            pass
        # self.bus.send(self.collect_msg,0.01)
        self.bus.send(msg,0.01) #time out 10ms
        # self.bus.send(silo_msg,0.01)
        
        # self.get_logger().info('Velocity transmit to STM32:[%f, %f, %f, %f]'%(self.V1,self.V2,self.V3,self.V4))
        for i in range (3):
        # while(finish_recv):
            try:
                can_msg =self.bus.recv(0.01)
                if(can_msg != None):            
                    if can_msg.arbitration_id == 0x407:
                        self.V1_back = ((can_msg.data[0] << 8) | can_msg.data[1])
                        self.V2_back = ((can_msg.data[2] << 8) | can_msg.data[3])
                        self.Omega_back = ((can_msg.data[4] << 8) | can_msg.data[5])
                        self.LaserX = (can_msg.data[6])
                        self.LaserY = (can_msg.data[7])
                        # self.goal_silo = can_msg.data[6]
                        self.W1=float(map(self.V1_back,0,65535,-100.0,100.0))
                        self.W2=float(map(self.V2_back,0,65535,-100.0,100.0))
                        self.laserX = float(map(self.LaserX,0,255,0.0,10.0))
                        self.laserY = float(map(self.LaserY,0,255,0.0,10.0))
                        # self.Omega_back = float(map(self.Omega_back,0,65535,-100,100))
                        self.Omega_back = float(map(self.Omega_back,0,65535,-3.14159,3.14159))
                        if (self.W1 < 0.01 and self.W1 > - 0.01):
                            self.W1= 0.0
                        if (self.W2 < 0.01 and self.W2 > - 0.01):
                            self.W2 = 0.0
                        if (self.Omega_back < 0.001 and self.Omega_back > - 0.001):
                            self.Omega_back = 0.0
                        
                    elif can_msg.arbitration_id == 0x215:
                        self.V1 = ((can_msg.data[0] << 8) | can_msg.data[1])
                        self.V2 = ((can_msg.data[2] << 8) | can_msg.data[3])
                        self.V1_encoder=float(map(self.V1,0,65535,-60,60))
                        self.V2_encoder=float(map(self.V2,0,65535,-60,60))
                        if (self.V1_encoder < 0.01 and self.V1_encoder > -0.01):
                            self.V1_encoder = 0.0
                        if (self.V2_encoder < 0.01 and self.V2_encoder > -0.01):
                            self.V2_encoder = 0.0
                        
                    elif can_msg.arbitration_id == 0x211:
                        self.V3 = ((can_msg.data[0] << 8) | can_msg.data[1])
                        self.V4 = ((can_msg.data[2] << 8) | can_msg.data[3])        
                        self.V3_encoder=float(map(self.V3,0,65535,-60,60))
                        self.V4_encoder=float(map(self.V4,0,65535,-60,60)) 
                        if (self.V3_encoder < 0.01 and self.V3_encoder > -0.01):
                            self.V3_encoder = 0.0
                        if (self.V4_encoder < 0.01 and self.V4_encoder > -0.01):
                            self.V4_encoder = 0.0

                    # elif can_msg.arbitration_id == 0x217:
                    #     self.goal_ball = can_msg.data[0]
                    #     ball.data = self.goal_ball
                    #     self.shoot_pub.publish(ball)
                        

                else:
                    self.get_logger().error('time out on msg recv!')
                    # finish_recv = False
                
            except can.CanOperationError:
                pass
        # control = Float32MultiArray()
        # control.data = [self.V1_encoder, self.V2_encoder, self.V3_encoder, self.V4_encoder]
        # self.encoder_pub.publish(control)
        
        
        # self.get_logger().info('Velocity Recieve from STM32:[%f, %f, %f, %f]'%(self.V1_encoder,self.V2_encoder,self.V3_encoder,self.V4_encoder))
        
    def back_callback(self):
        current = Float32MultiArray()
        laser = Float32MultiArray()
        laser.data = [self.laserX,self.laserY]
        self.laser_pub.publish(laser)
        # measurement
        # self.phi = self.phi + self.Omega_back*self.dt
        # if (self.phi > 2*np.pi or self.phi < -2*np.pi):
        #     self.phi = 0.0
        # if (self.phi >= np.pi):
        #     self.phi -= 2*np.pi
        # elif (self.phi <= -np.pi):
        #        self.phi += 2*np.pi
        self.Vx_ex = self.W1*np.cos(self.Yaw_en) - self.W2*np.sin(self.Yaw_en)
        self.Vy_ex = self.W1*np.sin(self.Yaw_en) + self.W2*np.cos(self.Yaw_en)
        self.X_en = self.X_en + self.Vx_ex*self.dt
        self.Y_en = self.Y_en + self.Vy_ex*self.dt
        self.Yaw_en = self.Omega_back
        # measurement = np.array([[self.X_en],[self.Y_en],[self.Yaw_en]])

        # state est from wheel equation
        # V = self.omni.forward_kinematic(self.V1_encoder,self.V2_encoder,self.V3_encoder,self.V4_encoder,self.states[2,0],"numpy")
        # X_next = self.states + np.array([[V[0]],[V[1]],[V[2]]])*self.dt
        # self.states = X_next
        # if (self.states[2,0] > 2*np.pi or self.states[2,0] < -2*np.pi):
        #     self.states[2,0] = 0.0
            
        # if (self.states[2,0] >= np.pi):
        #         self.states[2,0] -= 2*np.pi  
        # elif (self.states[2,0] <= -np.pi):
        #        self.states[2,0] += 2*np.pi
        # if (self.Y_en >= -5.6 and self.Y_en <= -5.3):
        #     self.TxData2[0] = 1
        # else:
        #     self.TxData2[0] = 2
        # if ((self.laserX >= 0.1 and self.laserX <=0.35) ):
        #      self.TxData2[0] = 1
        # else:
        #      self.TxData2[0] = 2
        # State_pred,P_pred = self.EKF.ekf_predicted(self.states,self.process_noise,self.P)
        # State_Update,P_update = self.EKF.ekf_update(State_pred,P_pred,measurement,self.measurement_noise)
        
        # self.states[0,0] = State_Update[0,0]
        # self.states[1,0] = State_Update[1,0]
        # self.states[2,0] = State_Update[2,0]
        # self.P = P_update
        
        current.data = [self.X_en ,self.Y_en, self.Yaw_en]
        self.external_pub.publish(current)
        self.get_logger().info('Velocity Recieve from Rotary:[%f, %f, %f, %f, %f]'%(self.X_en,self.Y_en,self.Yaw_en,self.laserX,self.laserY))
       
        # self.get_logger().info('Velocity Recieve from Encoder:[%f, %f, %f]'%(self.states[0,0],self.states[1,0],self.states[2,0]))
        

def main(args=None):
    rclpy.init(args=args)
    ros = ros_node()
    rclpy.spin(ros)
    ros.destroy_node()
    rclpy.shutdown     
            
if __name__=='__main__':
    main()