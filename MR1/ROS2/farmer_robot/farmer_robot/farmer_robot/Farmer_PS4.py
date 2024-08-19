import rclpy
import time
from rclpy.node import Node
from std_msgs.msg import String, Int8, Float32,Float32MultiArray, UInt8, Int8MultiArray
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

def map(Input, min_input, max_input, min_output, max_output):
    value = ((Input - min_input)*(max_output-min_output)/(max_input - min_input) + min_output)
    return value
class PS4Control(Node):
    def __init__(self):
        super().__init__('PS4_Node')
        self.pub_timer = 0.01
        self.subscribe_joy = self.create_subscription(
            Joy, "joy", self.subscribe_ps4, 10)
        
        self.input_controls = self.create_publisher(
            Twist, "/cmd_vel", 10)
        self.ps4_timer = self.create_timer(self.pub_timer,self.ps4_callback)
        # self.ps4_timer2 = self.create_timer(self.pub_timer,self.ps42_callback)
        self.mode_pub = self.create_publisher(String, "/mode", 10)
        self.goal_pub = self.create_publisher(String, "/goal", 10)
        self.position_pub = self.create_publisher(UInt8, '/position', 10)
        self.concept_pub = self.create_publisher(Int8MultiArray, '/concept', 10)
        
        self.axes_list = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.button_list = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

        self.Vx = 0.0
        self.Vy = 0.0
        self.Omega = 0.0
        self.Speed = 0.0
        self.Speed_plus = 0
        self.Speed_minus = 0
        self.SpeedAngle = 0.0
        self.Position = 0
        self.Mode = 0
        self.Shooter = 0
        self.Stepper = 0
        self.Drop = 0
        self.Catch = 0
        self.Push = 0
        self.increase = 0
        self.Decrease = 0
        self.Push_manual = 0

    
        self.prev_time = time.time()
        
    def subscribe_ps4(self,joy):
        self.axes_list = joy.axes
        self.button_list = joy.buttons
        
    def ps4_callback(self):
        manual = Twist()
        goal = UInt8()
        concept_mode = Int8MultiArray()
        mode = String()
        
        curr_time = time.time()
        

        if ((self.button_list[4] == 1) and (curr_time - self.prev_time)>0.3):
            self.Mode +=1
            if (self.Mode >1):
                self.Mode = 0
            self.prev_time = curr_time
        if ((self.button_list[5] == 1) and (curr_time - self.prev_time) > 0.3):
            self.Speed_plus += 1
            if (self.Speed_plus > 6):
                self.Speed_plus = 0
            self.Speed = self.Speed_plus - self.Speed_minus

            self.prev_time = curr_time

        # Decrease speed motor
        if ((self.button_list[7] == 1) and (curr_time - self.prev_time) > 0.3):
            self.Speed_minus += 1
            if (self.Speed_minus > 6):
                self.Speed_minus = 0
            self.Speed = self.Speed_plus - self.Speed_minus
            self.prev_time = curr_time
                    
            self.prev_time = curr_time
        if ((self.button_list[0] == 1) and (curr_time - self.prev_time)>0.3):
            self.Position +=1
            if (self.Position > 9):
                self.Position = 0
            goal.data = self.Position
            self.position_pub.publish(goal)
            self.prev_time = curr_time
        if ((self.button_list[1] == 1) and (curr_time - self.prev_time)>0.2):
            self.Shooter +=1
            if (self.Shooter > 4):
                self.Shooter = 0
            self.prev_time = curr_time
        if ((self.button_list[2] == 1)):
            self.Push = 1
        elif ((self.button_list[2] == 0)):
            self.Push = 0
        if ((self.button_list[3] == 1) ):
            self.Drop = 1
            # if (self.Drop > 6):
            #     self.Drop = 0
            self.prev_time = curr_time
        elif ((self.button_list[3] == 0)):
            self.Drop = 0
        if ((self.button_list[6] == 1)):
            self.Catch = 1
        else:
            self.Catch = 0
        if (self.axes_list[7] == 1.0):
            self.Push_manual = 1
        elif (self.axes_list[7] == -1.0):
            self.Push_manual = -1
        elif (self.axes_list[7] == 0.0 ):
            self.Push_manual = 0
        if (self.axes_list[6] == 1.0):
            self.Stepper = 1
        elif (self.axes_list[6] == -1.0):
            self.Stepper = -1
        elif (self.axes_list[6] == 0.0):
            self.Stepper = 0
        if (self.axes_list[1] == 1.0 or self.axes_list[1]== -1.0):
            self.Vx = (self.axes_list[1])*(0.8 + (self.Speed/10))
            # if ((self.Vx < 1.0 and self.Vx > -1.0) or (self.Vx > 3.0 or self.Vx < -3.0)):
            #     self.Vx = 0.0
            manual.linear.x = self.Vx
            self.input_controls.publish(manual)
            # self.get_logger().info('Manual control - Linear Velocity X : %f, Linear Velocity Y : %f, Angular Velocity Z : %f' % (self.Vx, self.Vy,self.Omega))
        elif (self.axes_list[1] == 0.0):
            self.Vx = 0.0
            manual.linear.x = self.Vx
            self.input_controls.publish(manual)
            # self.get_logger().info('Manual control - Linear Velocity X : %f, Linear Velocity Y : %f, Angular Velocity Z : %f' % (self.Vx, self.Vy,self.Omega))
        if (self.axes_list[0] == 1.0 or self.axes_list[0] == -1.0):
            self.Vy = (self.axes_list[0])*(0.8 + (self.Speed/10))
            # if ((self.Vy < 1.0 and self.Vy > -1.0) or (self.Vy > 3.0 or self.Vy < -3.0)):
            #     self.Vy = 0.0
            manual.linear.y = self.Vy
            self.input_controls.publish(manual)
            # self.get_logger().info('Manual control - Linear Velocity X : %f, Linear Velocity Y : %f, Angular Velocity Z : %f' % (self.Vx, self.Vy,self.Omega))
        elif (self.axes_list[0] == 0.0):
            self.Vy = 0.0
            manual.linear.y = self.Vy
            self.input_controls.publish(manual)
            # self.get_logger().info('Manual control - Linear Velocity X : %f, Linear Velocity Y : %f, Angular Velocity Z : %f' % (self.Vx, self.Vy,self.Omega))
        if (self.axes_list[3] == 1.0 or self.axes_list[3] == -1.0):
            self.Omega = (self.axes_list[3])*(0.8 + (self.Speed/10))
            # if ((self.Omega < 1.0 and self.Omega > -1.0) or (self.Omega > 3.14 or self.Omega < -3.14)):
            #     self.Omega = 0.0
            manual.angular.z = self.Omega
            self.input_controls.publish(manual)
            # self.get_logger().info('Manual control - Linear Velocity X : %f, Linear Velocity Y : %f, Angular Velocity Z : %f' % (self.Vx, self.Vy,self.Omega))
        elif ( self.axes_list[3] == 0.0):
            self.Omega = 0.0
            manual.angular.z = self.Omega
            self.input_controls.publish(manual)
        if (self.Mode == 0):
            mode.data = "manual"
        elif (self.Mode == 1):
            mode.data = "auto"
        self.mode_pub.publish(mode)
            # self.mode_pub.publish(mode)
        self.get_logger().info('Manual control - Linear Velocity X : %f, Linear Velocity Y : %f, Angular Velocity Z : %f' % (self.Vx, self.Vy,self.Omega))
        self.get_logger().info('Speed:%d, Position:%d, Mode:%d, Shooter:%d, Drop:%d, Push:%d, Catch:%d, Push_manual:%d, Stepper:%d' %(self.Speed, self.Position, self.Mode, self.Shooter, self.Drop, self.Push, self.Catch, self.Push_manual, self.Stepper))
        concept_mode.data = [self.Push, self.Catch, self.Drop, self.Push_manual, self.Shooter, self.Stepper] 
        
        
        self.concept_pub.publish(concept_mode)
    # def ps42_callback(self):
        
        
        
        
def main(args=None):
    rclpy.init(args=args)
    PS4 = PS4Control()
    rclpy.spin(PS4)
    PS4.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()