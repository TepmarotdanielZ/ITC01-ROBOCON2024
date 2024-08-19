## By: YIN CHHEANYUN ##
## Date: 4 jan 2023 ##

import rclpy
from rclpy.node import Node # type: ignore
from geometry_msgs.msg import Twist 
from std_msgs.msg import Float32MultiArray, Int8MultiArray, Float32, UInt8MultiArray, UInt32, String
import can
import numpy as np
from numpy.random import randn
from sensor_msgs.msg import Joy
from farmer_robot.mecanum_farmer import FarmerModel
import time
# from my_package_py.EKF import Extended_kalmanFilter
#Parameter
R = 0.05 #Radius of Wheel[m]
lx = 0.165 #length from wheel axis X to center robot [m]
ly = 0.225 #length from wheel axis Y to center robot [m]

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
        self.laser_pub = self.create_publisher(Float32MultiArray, '/laser', 10)

        #Subscriber Twist/cmd_vel
        self.Input_ = self.create_subscription(Float32MultiArray, '/control', self.sub_control, 10)
        self.position_motor_sub = self.create_subscription(Float32, '/motor_pos', self.pos_motor_callback, 10)
        self.concept_pub = self.create_subscription(Int8MultiArray, '/concept', self.concept_mode_callback, 10)
        self.state_sub = self.create_subscription(UInt8MultiArray, '/state', self.state_callback, 10)
        #Publish CANBUS to STM32
        self.timer = 0.001
        self.bus =can.interface.Bus(channel='can0', interface ='socketcan', bitrate=1000000)
        self.can_timer_ = self.create_timer(self.timer, self.timerCanCB)

        # self.time_position = self.create_timer(0.01, self.timer_position_Callback)
        #self.publisher= self.create_publisher(String, 'velocity',10)
        # self.time =self.create_timer(self.pub_timer, self.pub_msg)

        ## Mode
        self.R = 0.075
        self.Lx = 0.75/2
        self.Ly = 0.75/2 
        self.mecanum=FarmerModel(self.R,self.Lx,self.Ly)
        self.sampling_time = 0.01
        #Subscription from Joy or cmd_vel to CAN_Tx
        
        self.Omega = 0.0
        self.Speed = 0.0
        self.SpeedAngle = 0.0
        self.i=0
        self.TxData = [128,0,128,0,128,0,128,0]
        self.TxData1 = [128, 0, 0, 0, 0, 127, 0, 127]
        self.TxData2 = [0, 0]
        # self.V_back=[0,0,0,0]
        self.V1_input=0
        self.V2_input=0
        self.V3_input=0
        self.V4_input=0
        ###** Recieve from CAN_Rx to Publish check Position
        self.cpr = 1440
        self.r = 0.03
        self.first_init = True
        self.prev_tick = np.array([0,0])
        self.curr_tick = np.array([0,0])
        self.rotary = np.zeros(2)
        self.V_Back=[0,0,0,0]
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
        self.LaserX = 0.0
        self.LaserY = 0.0
        self.X= 0.0
        self.Y = 0.0
        self.Yaw = 0.0
        self.dt = 0.01 # 10ms
        self.rotary_old = 0
        self.x = 0.0

        self.X_en =  0.0
        self.Y_en = 0.0
        self.Yaw_en = 0.0
        self.X_m = 0.0
        self.Y_m = 0.0
        self.Yaw_m = 0.0
        self.u1 = 0.0
        self.u2 = 0.0
        self.u3 = 0.0
        self.u4 = 0.0
        self.motor_pos = 0.0
        self.concept_state = 0 
        self.state_call = 0
        # self.state = np.array([[self.X, self.Y, self.Yaw]])
    
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
    
    def pos_motor_callback(self,motor):
        self.motor_pos = int(map(motor.data, -10.0, 10.0, 0, 65535))
        self.TxData1[0] = ((self.motor_pos & 0xFF00) >> 8)
        self.TxData1[1] = (self.motor_pos & 0x00FF)
        self.concept_state = 1
    def state_callback(self, state):
        self.TxData2[0] = state.data[0]
        self.TxData2[1] = state.data[1]
        self.msg_state = can.Message(arbitration_id=0x103,  is_extended_id=False, dlc=2, data= self.TxData2)
        self.state_call = 1
    def concept_mode_callback(self, concept):
        self.TxData1[2] = int(concept.data[0])
        self.TxData1[3] = int(concept.data[1])
        self.TxData1[4] = int(concept.data[2])
        self.TxData1[5] = int(map(concept.data[3], -1.0, 1.0, 0, 255))
        self.TxData1[6] = int(concept.data[4])
        self.TxData1[7] = int(map(concept.data[5], -1.0, 1.0, 0, 255))
        self.concept_state = 1
    ###*** Publisher CAN_BUS to STM32 ***### 
    def timerCanCB(self):
        
        external = Float32MultiArray()
        laser  = Float32MultiArray()
        msg = can.Message(arbitration_id=0x101, is_extended_id=False, dlc=8, data=self.TxData)
        msg_concept = can.Message(arbitration_id=0x102, is_extended_id=False, dlc=8, data= self.TxData1)
        try:
            if (self.concept_state == 1):
                self.bus.send(msg_concept, 0.01)
                self.concept_state = 0
            elif(self.state_call == 1):
                self.bus.send(self.msg_state, 0.01)
                self.state_call = 0
        except can.CanError :
            self.get_logger().error('message not send')
            pass
        self.bus.send(msg,0.01) #time out 10ms
        # self.get_logger().info('Velocity transmit to STM32:[%f, %f, %f]'%(self.Vx,self.Vy,self.Vyaw))
        for i in range(3):
            try:
                can_msg =self.bus.recv(0.01)
                if(can_msg != None):            
                    if can_msg.arbitration_id == 0x407:
                        ADC_1 = ((can_msg.data[0] << 8) | can_msg.data[1])
                        ADC_2 = ((can_msg.data[2] << 8) | can_msg.data[3])
                        self.LaserX = ((ADC_1 *0.01416) + 0.1963)/10
                        self.LaserY = ((ADC_2 *0.01289) + 0.3364)/10
                        laser.data = [self.LaserX, self.LaserY]
                        self.laser_pub.publish(laser)

                    elif can_msg.arbitration_id == 0x215:
                        self.V1 = ((can_msg.data[0] << 8) | can_msg.data[1])
                        self.V2 = ((can_msg.data[2] << 8) | can_msg.data[3])
                        self.V1_encoder=float(map(self.V1,0,65535,-60.0,60.0))
                        self.V2_encoder=float(map(self.V2,0,65535,-60.0,60.0))
                        if (self.V1_encoder < 0.01 and self.V1_encoder > -0.01):
                            self.V1_encoder = 0.0
                        if (self.V2_encoder < 0.01 and self.V2_encoder > -0.01):
                            self.V2_encoder = 0.0
                        
                    elif can_msg.arbitration_id == 0x211:
                        self.V3 = ((can_msg.data[0] << 8) | can_msg.data[1])
                        self.V4 = ((can_msg.data[2] << 8) | can_msg.data[3])        
                        self.V3_encoder=float(map(self.V3,0,65535,-60.0,60.0))
                        self.V4_encoder=float(map(self.V4,0,65535,-60.0,60.0)) 
                        if (self.V3_encoder < 0.01 and self.V3_encoder > -0.01):
                            self.V3_encoder = 0.0
                        if (self.V4_encoder < 0.01 and self.V4_encoder > -0.01):
                            self.V4_encoder = 0.0
    
                else:
                    self.get_logger().error('time out on msg recv!')
                
            except can.CanOperationError:
                pass
        control = Float32MultiArray()
        control.data = [self.V1_encoder, self.V2_encoder, self.V3_encoder, self.V4_encoder]
        self.encoder_pub.publish(control)
        # self.get_logger().info('Velocity Recieve from STM32:[%f, %f, %f, %f]'%(self.V1_encoder,self.V2_encoder,self.V3_encoder,self.V4_encoder))
        self.get_logger().info('Sensor Feedback : W1: %f, W2: %f, Yaw: %f, LaserX:%f, LaserY:%f' %(self.W1,self.W2, self.Omega_back, self.LaserX, self.LaserY))

    # def timer_position_Callback(self):
    #     V = self.mecanum.forward_kinematic(self.V1_encoder, self.V2_encoder, self.V3_encoder, self.V4_encoder,0.0, "numpy")
    #     self.X_m = self.X_m + V[0]*self.dt
    #     self.Y_m = self.Y_m + V[1]*self.dt
    #     self.Yaw_m = self.Yaw_m + V[2]*self.dt
    #     if (self.Yaw_m>2*np.pi and self.Yaw_m < -2*np.pi):
    #         self.Yaw_m = 0.0
    #     if (self.Yaw_m > np.pi):
    #         self.Yaw_m -= 2*np.pi
    #     elif (self.Yaw_m < np.pi):
    #         self.Yaw_m += 2*np.pi
        # self.get_logger().info('Odometry_Wheel: X: %f, Y: %f, Yaw:%f' %(self.X_m, self.Y_m, self.Yaw_m))
def main(args=None):
    rclpy.init(args=args)
    ros = ros_node()
    rclpy.spin(ros)
    ros.destroy_node()
    rclpy.shutdown     
            
if __name__=='__main__':
    main()