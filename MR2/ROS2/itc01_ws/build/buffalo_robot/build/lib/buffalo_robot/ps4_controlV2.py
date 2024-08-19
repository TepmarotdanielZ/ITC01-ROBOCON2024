import rclpy
import time
from rclpy.node import Node
from std_msgs.msg import String, Int8, Float32,Float32MultiArray
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

def map(Input, min_input, max_input, min_output, max_output):
    value = ((Input - min_input)*(max_output-min_output)/(max_input - min_input) + min_output)
    return value
class PS4Control(Node):
    def __init__(self):
        super().__init__('PS4_Node')
        self.pub_timer = 0.05
        self.subscribe_joy = self.create_subscription(
            Joy, "joy", self.subscribe_ps4, 10)
        
        self.input_controls = self.create_publisher(
            Twist, "/cmd_vel", 10)
        self.ps4_timer = self.create_timer(self.pub_timer,self.ps4_callback)
        self.mode_pub = self.create_publisher(String, "/mode", 10)
        self.goal_pub = self.create_publisher(String, "/goal", 10)
        
        self.axes_list = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.button_list = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        self.prev_time = time.time()
        self.Vx = 0.0
        self.Vy = 0.0
        self.Omega = 0.0
        self.Speed = 0.0
        self.SpeedAngle = 0.0
        
    def subscribe_ps4(self,joy):
        self.axes_list = joy.axes
        self.button_list = joy.buttons
        
    def ps4_callback(self):
        manual = Twist()
        mode = String()
        goal = String()
        curr_time = time.time()
        if ((self.button_list[4] == 1)):
            mode.data = "manual"
            self.mode_pub.publish(mode)
            self.get_logger().info('Mode : "%s"' % (mode.data))
        if ((self.button_list[5] == 1)):
            mode.data = "auto"
            self.mode_pub.publish(mode)
            self.get_logger().info('Mode : "%s"' % (mode.data))
        if ((self.button_list[0] == 1)):
            goal.data = "success"
            self.goal_pub.publish(goal)
            self.get_logger().info('Goal : "%s"' % (goal.data))
        if ((self.button_list[1] == 1)):
            goal.data = "retry"
            self.goal_pub.publish(goal)
            self.get_logger().info('Goal : "%s"' % (goal.data))
        if ((self.button_list[2] == 1)):
            goal.data = "silo1"
            self.goal_pub.publish(goal)
            self.get_logger().info('Goal : "%s"' % (goal.data))
        if ((self.button_list[3] == 1)):
            goal.data = "silo2"
            self.goal_pub.publish(goal)
            self.get_logger().info('Goal : "%s"' % (goal.data))
        if ((self.button_list[11] == 1)):
            goal.data = "silo3"
            self.goal_pub.publish(goal)
            self.get_logger().info('Goal : "%s"' % (goal.data))
        if ((self.button_list[12] == 1)):
            goal.data = "silo4"
            self.goal_pub.publish(goal)
            self.get_logger().info('Goal : "%s"' % (goal.data))
        if ((self.button_list[7] == 1)):
            goal.data = "silo5"
            self.goal_pub.publish(goal)
            self.get_logger().info('Goal : "%s"' % (goal.data))
        if ((self.button_list[6] == 1)):
            goal.data = "ball"
            self.goal_pub.publish(goal)
            self.get_logger().info('Goal : "%s"' % (goal.data))
        if (self.axes_list[7] == 1.0 and (curr_time - self.prev_time)>0.2):
            self.Speed += 0.1
            if (self.Speed > 0.5):
                self.Speed = 0.0
            self.prev_time = curr_time
        elif (self.axes_list[7] == -1.0 and (curr_time - self.prev_time)>0.2):
            self.Speed -= 0.1
            if (self.Speed < -0.5):
                self.Speed = 0.0
            self.prev_time = curr_time
        elif (self.axes_list[7] == 0.0):
            self.Speed = self.Speed
        if (self.axes_list[6] == 1.0 and (curr_time - self.prev_time)>0.2):
            self.SpeedAngle += 0.1
            if (self.SpeedAngle > 0.5):
                self.SpeedAngle = 0.0
            self.prev_time = curr_time
        elif (self.axes_list[6] == -1.0 and (curr_time - self.prev_time)>0.2):
            self.SpeedAngle -= 0.1
            if (self.SpeedAngle < -0.5):
                self.SpeedAngle = 0.0
            self.prev_time = curr_time
        elif (self.axes_list[6] == 0.0):
            self.SpeedAngle = self.SpeedAngle
        # if (self.axes_list[7] == 1.0):
        #     self.Speed += 0.01
        # elif (self.axes_list[7] == -1.0):
        #     self.Speed -= 0.01
        # elif (self.axes_list[7] == 0.0 ):
        #     self.Speed = self.Speed
        # if (self.axes_list[6] == 1.0):
        #     self.SpeedAngle += 0.01
        # elif (self.axes_list[6] == -1.0):
        #     self.SpeedAngle -= 0.01
        # elif (self.axes_list[6] == 0.0):
        #     self.SpeedAngle = self.SpeedAngle
        if (self.axes_list[1] == 1.0 or self.axes_list[1]== -1.0):
            self.Vx = (self.axes_list[1])*(1.0 + self.Speed)
            if ((self.Vx < 0.7 and self.Vx > -0.7) or (self.Vx > 3.0 or self.Vx < -3.0)):
                self.Vx = 0.0
            manual.linear.x = self.Vx
            self.input_controls.publish(manual)
            self.get_logger().info('Manual control - Linear Velocity X : %f, Linear Velocity Y : %f, Angular Velocity Z : %f' % (self.Vx, self.Vy,self.Omega))
        elif (self.axes_list[1] == 0.0):
            self.Vx = 0.0
            manual.linear.x = self.Vx
            self.input_controls.publish(manual)
            self.get_logger().info('Manual control - Linear Velocity X : %f, Linear Velocity Y : %f, Angular Velocity Z : %f' % (self.Vx, self.Vy,self.Omega))
        if (self.axes_list[0] == 1.0 or self.axes_list[0] == -1.0):
            self.Vy = (self.axes_list[0])*(1.0 + self.Speed)
            if ((self.Vy < 0.7 and self.Vy > -0.7) or (self.Vy > 3.0 or self.Vy < -3.0)):
                self.Vy = 0.0
            manual.linear.y = self.Vy
            self.input_controls.publish(manual)
            self.get_logger().info('Manual control - Linear Velocity X : %f, Linear Velocity Y : %f, Angular Velocity Z : %f' % (self.Vx, self.Vy,self.Omega))
        elif (self.axes_list[0] == 0.0):
            self.Vy = 0.0
            manual.linear.y = self.Vy
            self.input_controls.publish(manual)
            self.get_logger().info('Manual control - Linear Velocity X : %f, Linear Velocity Y : %f, Angular Velocity Z : %f' % (self.Vx, self.Vy,self.Omega))
        if (self.axes_list[3] == 1.0 or self.axes_list[3] == -1.0):
            self.Omega = (self.axes_list[3])*(1.3 + self.SpeedAngle)
            if ((self.Omega < 0.7 and self.Omega > -0.7) or (self.Omega > 3.14 or self.Omega < -3.14)):
                self.Omega = 0.0
            manual.angular.z = self.Omega
            self.input_controls.publish(manual)
            self.get_logger().info('Manual control - Linear Velocity X : %f, Linear Velocity Y : %f, Angular Velocity Z : %f' % (self.Vx, self.Vy,self.Omega))
        elif ( self.axes_list[3] == 0.0):
            self.Omega = 0.0
            manual.angular.z = self.Omega
            self.input_controls.publish(manual)
            self.get_logger().info('Manual control - Linear Velocity X : %f, Linear Velocity Y : %f, Angular Velocity Z : %f' % (self.Vx, self.Vy,self.Omega))
def main(args=None):
    rclpy.init(args=args)
    PS4 = PS4Control()
    rclpy.spin(PS4)
    PS4.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()