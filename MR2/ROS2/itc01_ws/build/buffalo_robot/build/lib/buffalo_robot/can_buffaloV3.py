import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist , Vector3
from std_msgs.msg import Float32MultiArray,Float32, String, Int8, UInt16, UInt8MultiArray, UInt8
from sensor_msgs.msg import Imu
import can
import numpy as np
from sensor_msgs.msg import Joy
import time
from buffalo_robot.EKF import Extended_kalmanFilter
from numpy.random import randn
from buffalo_robot.omni_buffalo import Omni_model


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
        self.silo_pub = self.create_publisher(Int8, '/silo_goal' , 10)
        self.laser_pub = self.create_publisher(Float32MultiArray, '/laser', 10)
        self.ball_aready_pub = self.create_publisher(UInt8MultiArray, '/ball_already', 10)
        # self.back_timer = self.create_timer(self.pub_timer, self.back_callback)

        #Subscriber Twist/cmd_vel
        self.Input_ = self.create_subscription(Float32MultiArray, '/control', self.sub_control, 10)
        self.Input_ 
        self.collect = self.create_subscription(Float32MultiArray, '/ball_distance', self.collect_control, 10)
        self.state_sub = self.create_subscription(UInt16 , '/state', self.state_callback , 10)
        self.sub_typeball = self.create_subscription(String, '/ball_class', self.balltype_callback, 10)
        self.motor_sub = self.create_subscription(UInt8, '/motor',self.motor_callback, 10)
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
        self.TxData1 = [128,0]
        self.TxData2 = [0, 0]
        # self.V_back=[0,0,0,0]
        self.V1_input=0.0
        self.V2_input=0.0
        self.V3_input=0.0
        self.V4_input=0.0
        ###** Recieve from CAN_Rx to Publish check Position
        self.button = 0
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
        Lx = 0.30/2
        Ly = 0.30/2
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
        # self.state_est = np.array([[self.X_est],[self.Y_est],[self.Yaw_est]])
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
        self.ball_type = None
        self.ball_aready = 0
        self.check_three_ball  = 0
    
    ###*** Subscriber Twist msg ***###
    def balltype_callback(self,ball):
        self.ball_type = ball.data
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
        # self.d = d.data[0] 
        if (self.ball_type == "no_ball"):
            self.d = 0.0
        else :
            self.d = d.data[0]
        self.distance = int(map(self.d, 0.0, 100.0, 0, 65535))
        self.TxData1[0] = ((self.distance & 0xFF00) >> 8)
        self.TxData1[1] = (self.distance & 0x00FF)
        # self.TxData1[0] = self.distance
        self.collect_msg = can.Message(arbitration_id=0x111, data=self.TxData1, dlc=2, is_extended_id=False)
        self.collect_ball = 1
        self.get_logger().info('Ball distance:[%f]'%(d.data[0]))
    def state_callback(self, state):
        # self.state = state.data
        self.TxData2[0] = state.data
        self.shoot_msg = can.Message(arbitration_id=0x222, data=self.TxData2, dlc=2, is_extended_id=False)
        self.shooter_ball = 1
    def motor_callback(self, motor):
        self.TxData2[1] = motor.data
    ###*** Publisher CAN_BUS to STM32 ***### 
    def timerCanCB(self):
        current = Float32MultiArray()
        laser = Float32MultiArray()
        ball_ok = UInt8MultiArray()
        msg = can.Message(arbitration_id=0x103, data=self.TxData, dlc=8, is_extended_id=False)
       
        try :
            if (self.collect_ball == 1):
                self.collect_ball = 0
                self.bus.send(self.collect_msg,0.01)
            elif (self.shooter_ball == 1):
                self.shooter_ball = 0
                self.bus.send(self.shoot_msg,0.01)
        except can.CanError :
            self.get_logger().error('message not send')
            pass
        self.bus.send(msg,0.01) #time out 10ms
        
        # self.get_logger().info('Velocity transmit to STM32:[%f, %f, %f, %f]'%(self.V1,self.V2,self.V3,self.V4))
        for i in range (2):
            try:
                can_msg =self.bus.recv(0.01)
                if(can_msg != None):            
                    if can_msg.arbitration_id == 0x407:
                        self.V1_back = ((can_msg.data[0] << 8) | can_msg.data[1])
                        self.V2_back = ((can_msg.data[2] << 8) | can_msg.data[3])
                        self.Omega_back = ((can_msg.data[4] << 8) | can_msg.data[5])
                        self.W1= float(map(self.V1_back,0,65535,-100.0,100.0))
                        self.W2= float(map(self.V2_back,0,65535,-100.0,100.0))
                        self.Omega_back = float(map(self.Omega_back,0,65535,-3.14159,3.14159))
                        if (self.W1 < 0.005 and self.W1 > - 0.005):
                            self.W1= 0.0
                        if (self.W2 < 0.005 and self.W2 > - 0.005):
                            self.W2 = 0.0
                        if (self.Omega_back < 0.001 and self.Omega_back > - 0.001):
                            self.Omega_back = 0.0
                        current.data = [self.W1 ,self.W2, self.Omega_back]
                        self.external_pub.publish(current)
                    elif can_msg.arbitration_id == 0x409:
                        self.ball_aready = can_msg.data[0]
                        # self.LaserX = ((can_msg.data[1] << 8) | can_msg.data[2])
                        # self.LaserY = ((can_msg.data[3] << 8) | can_msg.data[4])
                        ADC_X = ((can_msg.data[1] << 8) | can_msg.data[2])
                        ADC_Y = ((can_msg.data[3] << 8) | can_msg.data[4])
                        # self.laserX = float(map(self.LaserX, 0, 65535, 0.0, 10.0))
                        # self.laserY = float(map(self.LaserY, 0, 65535, 0.0, 10.0))
                        self.laserX = ((ADC_X*0.08502) + 1.727)/100
                        # self.laserY = ((ADC_Y*0.08615) + 3.84)/100   # When Red
                        self.laserY = ((ADC_Y*0.154) - 2.169)/100  # When Blue
                        self.button = can_msg.data[5]
                        self.check_three_ball = can_msg.data[6]
                        ball_ok.data = [self.ball_aready, self.button, self.check_three_ball]
                        laser.data= [self.laserX, self.laserY]
                        self.ball_aready_pub.publish(ball_ok)
                        self.laser_pub.publish(laser)
                    # elif can_msg.arbitration_id == 0x215:
                    #     self.V1 = ((can_msg.data[0] << 8) | can_msg.data[1])
                    #     self.V2 = ((can_msg.data[2] << 8) | can_msg.data[3])
                    #     self.V1_encoder=float(map(self.V1,0,65535,-60,60))
                    #     self.V2_encoder=float(map(self.V2,0,65535,-60,60))
                    #     if (self.V1_encoder < 0.01 and self.V1_encoder > -0.01):
                    #         self.V1_encoder = 0.0
                    #     if (self.V2_encoder < 0.01 and self.V2_encoder > -0.01):
                    #         self.V2_encoder = 0.0
                        
                    # elif can_msg.arbitration_id == 0x211:
                    #     self.V3 = ((can_msg.data[0] << 8) | can_msg.data[1])
                    #     self.V4 = ((can_msg.data[2] << 8) | can_msg.data[3])        
                    #     self.V3_encoder=float(map(self.V3,0,65535,-60,60))
                    #     self.V4_encoder=float(map(self.V4,0,65535,-60,60)) 
                    #     if (self.V3_encoder < 0.01 and self.V3_encoder > -0.01):
                    #         self.V3_encoder = 0.0
                    #     if (self.V4_encoder < 0.01 and self.V4_encoder > -0.01):
                    #         self.V4_encoder = 0.0

                else:
                    self.get_logger().error('time out on msg recv!')
                
            except can.CanOperationError:
                pass
        
        self.get_logger().info('Velocity Recieve from Rotary:[%f, %f, %f, %f, %f, %d, %d, %d]'%(self.W1,self.W2,self.Omega_back,self.laserX,self.laserY, self.ball_aready, self.button, self.check_three_ball))
    # def back_callback(self):
    #     self.V = self.omni.forward_kinematic(self.V1_encoder,self.V2_encoder,self.V3_encoder,self.V4_encoder,self.Omega_back,"numpy")
        
    #     self.X_en = self.X_en + self.V[0]*self.dt
    #     self.Y_en = self.Y_en + self.V[1]*self.dt
    #     self.Yaw_en = self.Yaw_en + self.V[2]*self.dt
    #     if (self.Yaw_en > 2*np.pi or self.Yaw_en < -2*np.pi):
    #         self.Yaw_en = 0.0
            
    #     if (self.Yaw_en >= np.pi):
    #             self.Yaw_en -= 2*np.pi  
    #     elif (self.Yaw_en <= -np.pi):
    #            self.Yaw_en += 2*np.pi
        
    #     self.get_logger().info('Velocity Recieve from Motor:[%f, %f, %f, %f, %f]'%(self.X_en,self.Y_en,self.Yaw_en,self.laserX,self.laserY))
    

def main(args=None):
    rclpy.init(args=args)
    ros = ros_node()
    rclpy.spin(ros)
    ros.destroy_node()
    rclpy.shutdown     
            
if __name__=='__main__':
    main()