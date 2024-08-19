import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray,Float32,Int32,String
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
        ## Subscription velocity to manual
        self.manual_sub = self.create_subscription(Twist,'/manual', self.subscrib_manual, 10)

        ## Subscription Rotary and IMU Feedback
        self.feedback_sub = self.create_subscription(Float32MultiArray, '/external', self.subscrib_feedback, 10)
        self.imu_sub = self.create_subscription(Imu,'/imu',self.subscript_imu,10)
        ## Subscription from Encoder motor
        self.encoder_sub = self.create_subscription(Float32MultiArray, '/encoder', self.subscrib_encoder, 10)

        ## Subscription Mode
        self.mode_sub = self.create_subscription(String, '/mode', self.subscrib_mode, 10)

        ## Subscription Goal
        self.goal_sub = self.create_subscription(String, '/goal', self.subscrib_goal, 10)

        ## Publish input control manual
        self.input_pub = self.create_publisher(Float32MultiArray, '/control', 10)
        ## Publisher Output PID
        timer = 0.01
        self.output_pub = self.create_publisher(Float32MultiArray,'/pointer',10)
        self.output_timer = self.create_timer(timer,self.Calculate_position)

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
        self.R = 0.05
        self.Lx = 0.245/2
        self.Ly = 0.245/2 
        self.omni=Omni_model(self.R,self.Lx,self.Ly)
        self.sampling_time = 0.01
        self.mode_status = "manual"

        ## Variable PID
        self.dt = 0.01
        self.Kp = np.array([1.2, 1.2, 1.0])
        self.Ki = np.array([0.001, 0.001, 0.001])
        self.Kd = np.array([0.005, 0.005, 0.005])
        self.alpha = 0.5
        self.integral_min = -2.5
        self.integral_max = 2.5
        self.output_min = -2.5
        self.output_max = 2.5
        self.integral_min_Angular = -3.14
        self.integral_max_Angular = 3.14
        self.output_min_Angular = -3.14
        self.output_max_Angular = 3.14
        self.pid_X= PIDController(self.Kp[0], self.Ki[0], self.Kd[0], self.dt, self.alpha, self.integral_min, self.integral_max, self.output_min, self.output_max)
        self.pid_Y= PIDController(self.Kp[1], self.Ki[1], self.Kd[1], self.dt, self.alpha, self.integral_min, self.integral_max, self.output_min, self.output_max)
        self.pid_Yaw= PIDController(self.Kp[2], self.Ki[2], self.Kd[2], self.dt, self.alpha, self.integral_min_Angular, self.integral_max_Angular, self.output_min_Angular, self.output_max_Angular)
        self.output_vx = 0.0
        self.output_vy = 0.0
        self.output_omega = 0.0

        ## Variable Auto
        self.vx = 0.0
        self.vy = 0.0
        self.omega = 0.0
        self.X_start = 0.0
        self.Y_start = 0.0
        self.Yaw_start = 0.0
        self.Goal = np.array([0.0, 0.0, 0.0])
        self.goal0 = np.array([0.0, 0.0, 0.0])
        self.goal1 = np.array([0.0, 0.0, 0.0])
        self.goal2 = np.array([0.0, 0.0, 0.0])
        self.goal3 = np.array([0.0, 0.0, 0.0])
        self.goal4 = np.array([0.0, 0.0, 0.0])
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

        ## Variable sensor feedback
        self.Vx_ex = 0.0
        self.Vy_ex = 0.0
        self.yaw_imu = 0.0
        self.V_m = np.array([[0.0],[0.0],[0.0]])
        self.W1 = 0.0
        self.W2 = 0.0
        self.W3 = 0.0
        self.W4 = 0.0
    def subscrib_manual(self, V):
        self.Vx = V.linear.x
        self.Vy = V.linear.y
        self.Omega = V.angular.z
    def subscrib_mode(self, mode):
        self.mode_status = mode.data
        
    def subscrib_goal(self, goal):
        if (goal.data == "goal1"):
            self.goal1[0] = 3.0+0.0
            self.goal1[1] = 0.0
            self.goal1[2] = 0.0
            self.state_start = [self.current_x, self.current_y, self.current_yaw]
            self.state_end = self.goal1
            self.get_logger().info('Goal : "[%f, %f, %f]"' % (self.state_end[0], self.state_end[1], self.state_end[2]))
        elif (goal.data == "goal2"):
            self.goal2[0] = 3.0+0.2
            self.goal2[1] = -2.5
            self.goal2[2] = 0.0
            self.state_start = [self.current_x, self.current_y, self.current_yaw]
            self.state_end = self.goal2
            self.get_logger().info('Goal : "[%f, %f, %f]"' % (self.state_end[0], self.state_end[1], self.state_end[2]))
        elif (goal.data == "goal3"):
            self.goal3[0] = 0.0
            self.goal3[1] = 0.0
            self.goal3[2] = 0.0
            self.state_start = [self.current_x, self.current_y, self.current_yaw]
            self.state_end = self.goal3
            self.get_logger().info('Goal : "[%f, %f, %f]"' % (self.state_end[0], self.state_end[1], self.state_end[2]))
        # elif (goal.data == "goal4"):
        #     self.goal4[0] = 8.0
        #     self.goal4[1] = -1.7
        #     self.goal4[2] = 1.57
        #     self.state_start = [self.current_x, self.current_y, self.current_yaw]
        #     self.state_end = self.goal4
        #     self.get_logger().info('Goal : "[%f, %f, %f]"' % (self.state_end[0], self.state_end[1], self.state_end[2]))
        # elif (goal.data == "back1"):
        #     self.state_start = [self.current_x, self.current_y, self.current_yaw]
        #     self.goal0[0] = 0.0
        #     self.goal0[1] = 0.0
        #     self.goal0[2] = 0.0
        #     self.state_end = self.goal0
        #     self.get_logger().info('Goal : "[%f, %f, %f]"' % (self.state_end[0], self.state_end[1], self.state_end[2]))
            

    def subscrib_feedback(self, V_external):
        W1 = V_external.data[0]
        W2 = V_external.data[1]
        theta = V_external.data[2]
        self.Vx_ex = W1*np.cos(theta) - W2*np.sin(theta)
        self.Vy_ex = W1*np.sin(theta) + W2*np.cos(theta)
        self.yaw_imu = theta
        # if (self.Vx_ex < 0.05 and self.Vx_ex > -0.05):
        #     self.Vx_ex = 0.0
        # if (self.Vy_ex < 0.05 and self.Vy_ex > -0.05):
        #     self.Vy_ex = 0.0
        # if (self.yaw_imu < 0.05 and self.yaw_imu > -0.05):
        #     self.yaw_imu = 0.0
    
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
        
        self.errorX.append(self.state_end[0] - self.current_x)
        self.errorY.append(self.state_end[1] - self.current_y)
        self.errorYaw.append(self.state_end[2] - self.current_yaw)

        self.output_vx = self.pid_X.calculate_pid(self.errorX)
        self.output_vy = self.pid_X.calculate_pid(self.errorY)
        self.output_omega = self.pid_X.calculate_pid(self.errorYaw)

        # USE External Rotary
        self.current_x = self.current_x + self.Vx_ex*self.dt
        self.current_y = self.current_y + self.Vy_ex*self.dt
        self.current_yaw = self.yaw_imu

        # USE Encoder Motor
        # self.V_m = self.omni.forward_kinematic(self.W1,self.W2,self.W3,self.W4,self.current_yaw,"numpy")
        # self.current_x = self.current_x + self.V_m[0,0]*self.dt
        # self.current_y = self.current_y + self.V_m[1,0]*self.dt
        # self.current_yaw = self.current_yaw + self.V_m[2,0]*self.dt
        
        self.get_logger().info('Current : "[%f, %f, %f]"' % (self.current_x, self.current_y, self.current_yaw))
        
        if (self.mode_status == "manual"):
            V = Float32MultiArray()
            W = self.omni.inverse_kinematic(self.Vx,self.Vy,self.Omega,"numpy")
            V.data = [W[0,0],W[1,0],W[2,0],W[3,0]]  
            self.input_pub.publish(V)
            # self.get_logger().info('Velocity control : "[%f, %f, %f]"' % (self.Vx, self.Vy, self.Omega))
        elif (self.mode_status == "auto"):
            V = Float32MultiArray()
            W = self.omni.inverse_kinematic(self.output_vx,self.output_vy,self.output_omega,"numpy")
            V.data = [W[0,0],W[1,0],W[2,0],W[3,0]] 
            self.input_pub.publish(V)
            # self.get_logger().info('Goal : "[%f, %f, %f]"' % (self.output_vx, self.output_vy, self.output_omega))

def main(args=None):
    rclpy.init(args=args)
    pid_sim = PID_Simulation()
    rclpy.spin(pid_sim)
    pid_sim.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()