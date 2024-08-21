## By: YIN CHHEANYUN ##
## Date: 4 jan 2023 ##

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist 
from std_msgs.msg import Float32MultiArray, UInt16MultiArray, Int8, Int16, Int32, UInt16, UInt32, String
import can
import numpy as np
from sensor_msgs.msg import Joy

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
        # self.pub_timer = 0.01
        self.back_pub_ = self.create_publisher(Float32MultiArray, '/back',10)
        # self.back_timer = self.create_timer(self.pub_timer, self.back_callback)

        #Subscriber Twist/cmd_vel
        self.Input_ = self.create_subscription(Twist, '/cmd_vel', self.subCmdCB, 10)
        self.Input_ 

        #Publish CANBUS to STM32
        self.timer = 0.01
        self.bus =can.interface.Bus(channel='can0', interface ='socketcan', bitrate=1000000)
        self.can_timer_ = self.create_timer(self.timer, self.timerCanCB)

        #self.publisher= self.create_publisher(String, 'velocity',10)
        # self.time =self.create_timer(self.pub_timer, self.pub_msg)

        #Subscription from Joy or cmd_vel to CAN_Tx
        self.vx = 0.0
        self.vy = 0.0
        self.vyaw = 0.0
        self.Vx = 0.0
        self.Vy = 0.0
        self.Omega = 0.0
        self.Speed = 0.0
        self.SpeedAngle = 0.0
        self.i=0
        self.TxData = [128,0,128,0,128,0,128,0]
        # self.V_back=[0,0,0,0]

        ###** Recieve from CAN_Rx to Publish check Position
        self.V_Back=[0,0,0,0]
        self.V1=0.0
        self.V2=0.0
        self.V3=0.0
        self.V4=0.0
        self.Vx_back =0.0
        self.Vy_back = 0.0
        self.Omega_back = 0.0
        self.X_old = 0.0
        self.Y_old = 0.0
        self.Yaw_old = 0.0
        self.dt = 0.01 # 10ms
    
    ###*** Subscriber Twist msg ***###
    def subCmdCB(self,msg):
        self.vx =msg.linear.x
        self.vy =msg.linear.y
        self.vyaw=msg.angular.z
        self.Vx = int(map(self.vx,-1.5,1.5,0,65535))
        self.Vy = int(map(self.vy,-1.5,1.5,0,65535))
        self.Vyaw = int(map(self.vyaw,-3.14,3.14,0,65535))
        self.TxData[0] = ((self.Vx & 0xFF00) >> 8)
        self.TxData[1] = (self.Vx & 0x00FF)
        self.TxData[2] = ((self.Vy & 0xFF00) >> 8)
        self.TxData[3] = (self.Vy & 0x00FF)
        self.TxData[4] = ((self.Vyaw & 0xFF00) >> 8)
        self.TxData[5] = (self.Vyaw & 0x00FF)
        self.TxData[6] = ((self.Vx & 0xFF00) >> 8)
        self.TxData[7] = (self.Vx & 0x00FF)
        
    ###*** Publisher CAN_BUS to STM32 ***### 
    def timerCanCB(self):
        pub_msg=Float32MultiArray()
        msg = can.Message(arbitration_id=0x103, is_extended_id=False, data=self.TxData)
        self.bus.send(msg,0.01) #time out 10ms
        self.get_logger().info('Velocity transmit to STM32:[%f, %f, %f]'%(self.Vx,self.Vy,self.Vyaw))
        for i in range(1):
            try:
                can_msg =self.bus.recv(0.01)
                if(can_msg != None):            
                    if can_msg.arbitration_id == 0x407:
                        self.Vx_back = ((can_msg.data[0] << 8) | can_msg.data[1])
                        self.Vy_back = ((can_msg.data[2] << 8) | can_msg.data[3])
                        self.Omega_back = ((can_msg.data[4] << 8) | can_msg.data[5])
                else:
                    self.get_logger().error('time out on msg recv!')
                
            except can.CanOperationError:
                pass

        # Position of robot
        X = self.X_old + self.Vx_back*self.dt
        Y = self.Y_old + self.Vy_back*self.dt
        Yaw = self.Yaw_old + self.Omega_back*self.dt
        self.X_old = X
        self.Y_old = Y
        self.Yaw_old = Yaw
        pub_msg=Float32MultiArray()
        pub_msg.data=[X,Y,Yaw]
        self.get_logger().info('publish Velocity Receive From STM32: [%f,%f,%f]' 
                               %(pub_msg.data[0],pub_msg.data[1],pub_msg.data[2]))
        self.back_pub_.publish(pub_msg)
            
    
    ###*** Publisher Float32MultiArray ***###
    # def back_callback(self):
    #     # Rotation Matrix
    #     R = np.array([[np.cos(Yaw), np.sin(Yaw), 0.0],
    #                   [-np.sin(Yaw), np.cos(Yaw), 0.0],
    #                   [0.0 , 0.0, 1.0]])
    #     # Position of robot
    #     X = self.X_old + self.Vx_back*self.dt
    #     Y = self.Y_old + self.Vy_back*self.dt
    #     Yaw = self.Yaw_old + self.Omega_back*self.dt
    #     self.X_old = X
    #     self.Y_old = Y
    #     self.Yaw_old = Yaw
    #     pub_msg=Float32MultiArray()
    #     pub_msg.data=[X,Y,Yaw]
    #     self.get_logger().info('publish Velocity Receive From STM32: [%f,%f,%f]' 
    #                            %(pub_msg.data[0],pub_msg.data[1],pub_msg.data[2]))
    #     self.back_pub_.publish(pub_msg)
    
def main(args=None):
    rclpy.init(args=args)
    ros = ros_node()
    rclpy.spin(ros)
    ros.destroy_node()
    rclpy.shutdown     
            
if __name__=='__main__':
    main()