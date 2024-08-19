import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Int32
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray,Float32
import casadi as ca
import numpy as np
import matplotlib.pyplot as plt
import math
import time
import sys
import pathlib
from buffalo_robot.mpc_omni import MPC_solver
from buffalo_robot.omni_buffalo import Omni_model
from buffalo_robot.bezier_path import calc_bezier_path
from buffalo_robot.cubic_spline_planner import calc_spline_course
# from bezier_path import calc_4points_bezier_path
sys.path.append(str(pathlib.Path(__file__).parent.parent.parent))
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


class Simulation_MPC(Node):
     def __init__(self):
          super().__init__('MPC_SIM')

        #   self.control_sub = self.create_subscription(String, '/control', self.sub_control,10)
          timer = 0.1
          timer_transmit = 0.01
          self.timer_callback = self.create_timer(timer,self.callback_timer)
          self.manual_input = self.create_subscription(Twist, '/manual', self.manual_subsciption, 10)
          self.goal_sub = self.create_subscription(String,'/goal', self.subscript_goal,10)
          self.feedback_sub = self.create_subscription(Float32MultiArray, '/external', self.feedback_states, 10)
          self.velocity_sub = self.create_subscription(Float32MultiArray, '/encoder', self.velocity_back, 10)
          self.mode_sub = self.create_subscription(String, '/mode', self.change_mode, 10)
          self.pub_velocity = self.create_publisher(Float32MultiArray, '/control', 10)
          self.timer_velocity = self.create_timer(timer_transmit,self.velocity_transmit)

          #Setup Robot
          R = 0.06
          Lx = 0.45/2
          Ly = 0.45/2
          self.omni = Omni_model(R,Lx,Ly)

          #Setup MPC
          self.lowX = [-15.0, -10.0, -3.14]
          self.highX = [15.0, 10.0, 3.14]
          self.lowU = -30.0
          self.highU = 30.0
          Q = [1200, 1200, 800] # Use travel 3
          R = [1, 1, 1, 1]
        #   Q = [1500, 1200, 700]
        #   R = [0.01, 0.01, 0.01, 0.01]
          self.N = 50
          self.T = 0.1
          self.mpciter = 0
          self.N_IND_SEARCH = 10
          
          self.x_next = np.zeros(3)
          self.prev_dind = 0
          self.next_dind = 0
          self.index = 0
          self.pred_index = 0
          self.mpc = MPC_solver(self.lowX,self.highX,self.lowU,self.highU,Q,R,self.N,self.T)
          self.f,self.solver,self.args = self.mpc.setup_MPC()
          self.norm_cond = 0.0
          self.speed_up = lambda t: 30*(1-np.exp(-2*t))
          self.u1 = 0.0
          self.u2 = 0.0
          self.u3 = 0.0
          self.u4 = 0.0
          self.current_states = np.array([0.0, 0.0, 0.0], dtype=np.float64)
          self.current_controls = np.array([0.0, 0.0, 0.0, 0.0], dtype=np.float64)
          self.current_x = 0.0
          self.current_y = 0.0
          self.current_yaw = 0.0
          self.feedback_state = np.array([self.current_x,self.current_y,self.current_yaw], dtype=np.float64)
          self.mode_status = "auto"
          self.Vx = 0.0
          self.Vy = 0.0
          self.Omega = 0.0
          self.V = [0.0, 0.0, 0.0, 0.0]
          self.roll = 0.0
          self.pitch = 0.0
          self.yaw = 0.0
          # Path_Success:(goal1,goal2,goal3)
          # Path_Retry: (goal4,goal2,gaol3)
          # Path_Silo1: (goal5), Silo2:(goal6), Silo3:(goal7), Silo4:(goal8), Silo5:(goal9)
          # Path_Ball: (gaol10)
          self.game = "silo1"
          self.state = 0
          self.X_start = [0.0, 0.0, 0.0]
          self.X_End = [0.0, 0.0, 0.0]
          ### Test Lab
        #   self.X_end  = [5.2, 0.0, 0.0]    #  Retry_start
        #   self.X_end1 = [5.5, -2.85, -1.57]  # 
        #   self.X_end2 = [7.5, -2.85, 1.57]
        #   self.X_end3 = [7.5, -1.0, -1.57]  #Ball
        #   self.X_end  = [3.0, 0.0, 0.0]    #  Test in lab
        #   self.X_end1 = [3.50, -2.5, -0.0]  # 
        #   self.X_end2 = [3.0, 0.0, -1.57]
      #     self.X_end  = [5.0, 0.0, 0.0]    #  Retry_start
      #     self.X_end1 = [6.0, -3.2, -1.57]  # 
      #     self.X_end2 = [8.0, -3.5, 1.57]
        #   self.X_end  = [5.5, 0.0, 0.0]    #  Retry_start
        #   self.X_end1 = [6.1, -3.8, -1.57]  # 
        #   self.X_end2 = [8.5, -4.3, 1.57]
          self.X_end  = [5.7, 0.0, 0.0]    #  Retry_start
          self.X_end1 = [6.0, -3.8, -1.57]  # 
          self.X_end2 = [9.0, -4.0, 1.57]
          self.X_end3 = [10.0, -1.5, -1.57]  #Ball
          self.X_end4 = [8.1, -4.28, -1.57]  #Silo1
          self.X_end5 = [8.9, -4.28, -1.57]  # Silo2
          self.X_end6 = [9.65, -4.28, -1.57] # Silo3
          self.X_end7 = [10.4, -4.28, -1.57] # Silo4
          self.X_end8 = [11.15, -4.28, -1.57] # Silo5
          path, _ = self.calc_4points_bezier_path(self.X_start, self.X_End, 2.5, 50)
          self.goal0 = self.goal1 = np.vstack([path[:, 0], path[:, 1], np.append(np.arctan2(np.diff(path[:, 1]), np.diff(path[:, 0])), 0.0)])
          self.goal1 = [0]
          self.goal2 = [0]
          self.goal3 = [0]
          self.goal4 = [0]
          self.goal5 = [0]
          self.goal6 = [0]
          self.goal7 = [0]
          self.goal8 = [0]
          self.goal_states = np.hstack([self.goal0])
          
          self.n_points = 50
          self.states = np.tile(self.current_states.reshape(3,1), self.N+1)
          self.controls = np.tile(self.current_controls.reshape(4, 1), self.N)

          self.next_trajectories = np.tile(self.current_states.reshape(3, 1), self.N+1)
          self.next_controls = np.tile(self.current_controls.reshape(4, 1), self.N)

          self.hist_x = [self.current_states[0]]
          self.hist_y = [self.current_states[1]]
          self.hist_th = [self.current_states[2]]

          self.X_ref = self.goal_states[0,:]
          self.Y_ref = self.goal_states[1,:]
          self.Yaw_ref = self.smooth_yaw(self.goal_states[2,:])
          self.ax = 0
          self.ay = 0
          self.ax1 = 0
          self.ay1 = 0
          self.ax2 = 0
          self.ay2 = 0
          self.ax3 = 0
          self.ay3 = 0
          self.ax4 = 0
          self.ay4 = 0
          self.ax5 = 0
          self.ay5 = 0
          self.ax6 = 0
          self.ay6 = 0
          
          self.target_ind = self.calc_index_trajectory(self.current_states[0],self.current_states[1],self.X_ref,self.Y_ref,1)
          
     def calc_index_trajectory(self,state_x, state_y, cx, cy, pind):

        dx = [state_x - icx for icx in cx[pind:(pind + self.N_IND_SEARCH)]]
        dy = [state_y - icy for icy in cy[pind:(pind + self.N_IND_SEARCH)]]

        d = [idx ** 2 + idy ** 2 for (idx, idy) in zip(dx, dy)]


        mind = min(d)

        ind = d.index(mind) + pind

        return ind

     def smooth_yaw(self,yaw):

         for i in range(len(yaw) - 1):
                dyaw = yaw[i + 1] - yaw[i]

                while dyaw >= math.pi / 2.0:
                        yaw[i + 1] -= math.pi * 2.0
                        dyaw = yaw[i + 1] - yaw[i]

                while dyaw <= -math.pi / 2.0:
                        yaw[i + 1] += math.pi * 2.0
                        dyaw = yaw[i + 1] - yaw[i]

         return yaw

     def calc_4points_bezier_path(self,X_start, X_end, offset, n_points):
         dist = np.hypot(X_start[0] - X_end[0], X_start[1] - X_end[1]) / offset
         control_points = np.array(
                [[X_start[0], X_start[1]],
                [X_start[0] + dist * np.cos(X_start[2]), X_start[1] + dist * np.sin(X_start[2])],
                [X_end[0] - dist * np.cos(X_end[2]), X_end[1] - dist * np.sin(X_end[2])],
                [X_end[0], X_end[1]]])
         path = calc_bezier_path(control_points, n_points)

         return path, control_points 
     def sucess(self,dl):
        # Test lab
        # ax = [0.0, 2.0, 4.0, 4.8, 5.0, 5.0, 5.0, 6.2, 6.8, 7.2]
        # ay = [0.0, 0.0,-0.0,-0.2,-0.5, -1.5,-3.0,-3.0,-3.0,-1.0]
        # game field
        ax = [0.0, 2.0, 4.0, 5.7, 5.8, 5.9, 6.0, 6.0, 7.2, 8.3, 8.5, 9.0]
        ay = [0.0, 0.0,-0.0,-0.5,-0.7,-1.0,-1.5,-3.7,-4.0,-3.6,-3.3,-1.0]
        cx, cy, cyaw, ck, s = calc_spline_course(
            ax, ay, ds=dl)

        return cx, cy, cyaw, ck
     def retry(self,dl):
        ax = [4.0,5.5, 6.2, 8.5, 9.0]
        ay = [-0.0,-0.5, -3.8, -3.5, -1.0]
        cx, cy, cyaw, ck, s = calc_spline_course(
            ax, ay, ds=dl)

        return cx, cy, cyaw, ck
     def silo1(self,dl):
        ax = [0.0,3.0]
        ay = [0.0,0.0]
        cx, cy, cyaw, ck, s = calc_spline_course(
            ax, ay, ds=dl)

        return cx, cy, cyaw, ck
     def silo2(self,dl):
        ax = [self.current_x,0.0]
        ay = [self.current_y,0.0]
        cx, cy, cyaw, ck, s = calc_spline_course(
            ax, ay, ds=dl)

        return cx, cy, cyaw, ck
     def silo3(self,dl):
        ax = [self.current_x,9.65]
        ay = [self.current_y,-4.28]
        cx, cy, cyaw, ck, s = calc_spline_course(
            ax, ay, ds=dl)

        return cx, cy, cyaw, ck
     def silo4(self,dl):
        ax = [self.current_x,5.5, 6.2, 8.5, 9.0]
        ay = [self.current_y,-0.5, -3.2, -3.0, -1.0]
        cx, cy, cyaw, ck, s = calc_spline_course(
            ax, ay, ds=dl)

        return cx, cy, cyaw, ck
     def feedback_states(self,current):
           self.current_x = current.data[0]
           self.current_y = current.data[1]
           self.current_yaw = current.data[2]
           self.current_states = np.array([self.current_x, self.current_y, self.current_yaw], dtype=np.float64)
        
     def velocity_back(self, control):
           self.current_controls[0] = control.data[0]
           self.current_controls[1] = control.data[1]
           self.current_controls[2] = control.data[2]
           self.current_controls[3] = control.data[3]
     def change_mode(self, mode):
           self.mode_status = mode.data
     def manual_subsciption(self,input):
           self.Vx = input.linear.x
           self.Vy = input.linear.y
           self.Omega = input.angular.z
     def subscript_goal(self,goal):
          if (goal.data == "success"):
                self.X_ref, self.Y_ref, self.Yaw_ref, ck = self.sucess(0.1)
                self.Yaw_ref = self.smooth_yaw(self.Yaw_ref)
                self.target_ind = self.calc_index_trajectory(self.current_states[0],self.current_states[1],self.X_ref,self.Y_ref,1)
                self.goal_states = np.vstack([self.X_ref,self.Y_ref,self.Yaw_ref])
                self.ax = self.X_ref
                self.ay = self.Y_ref
          elif (goal.data == "retry"):
                self.X_ref, self.Y_ref, self.Yaw_ref, ck = self.retry(0.1)
                self.Yaw_ref = self.smooth_yaw(self.Yaw_ref)
                self.target_ind = self.calc_index_trajectory(self.current_states[0],self.current_states[1],self.X_ref,self.Y_ref,1)
                self.ax = self.X_ref
                self.ay = self.Y_ref
          elif (goal.data == "silo1"):
                self.X_ref, self.Y_ref, self.Yaw_ref, ck = self.silo1(0.1)
                self.Yaw_ref = self.smooth_yaw(self.Yaw_ref)
                self.target_ind = self.calc_index_trajectory(self.current_states[0],self.current_states[1],self.X_ref,self.Y_ref,1)
                self.ax1 = self.X_ref
                self.ay1 = self.Y_ref
          elif (goal.data == "silo2"):
                self.X_ref, self.Y_ref, self.Yaw_ref, ck = self.silo2(0.1)
                self.Yaw_ref = self.smooth_yaw(self.Yaw_ref)
                self.target_ind = self.calc_index_trajectory(self.current_states[0],self.current_states[1],self.X_ref,self.Y_ref,1)
                self.ax2 = self.X_ref
                self.ay2 = self.Y_ref
          elif (goal.data == "silo3"):
                self.X_ref, self.Y_ref, self.Yaw_ref, ck = self.silo3(0.1)
                self.Yaw_ref = self.smooth_yaw(self.Yaw_ref)
                self.target_ind = self.calc_index_trajectory(self.current_states[0],self.current_states[1],self.X_ref,self.Y_ref,1)
                self.ax3 = self.X_ref
                self.ay3 = self.Y_ref
          elif (goal.data == "silo4"):
                self.X_start = [self.current_x,self.current_y,self.current_yaw]
                path8, _ = self.calc_4points_bezier_path(self.X_start, self.X_end7, 4.0, 50)
                self.goal8 = np.vstack([path8[:, 0], path8[:, 1], np.append(np.arctan2(np.diff(path8[:, 1]), np.diff(path8[:, 0])), -1.57)])
                self.goal_states = np.hstack([self.goal8])
                self.X_start = self.current_x
                self.Y_start = self.current_y
                self.Yaw_start = self.current_yaw
                self.X_ref = self.goal_states[0, :]
                self.Y_ref = self.goal_states[1, :]
                self.Yaw_ref = self.smooth_yaw(self.goal_states[2, :])
                self.target_ind = self.calc_index_trajectory(self.current_states[0],self.current_states[1],self.X_ref,self.Y_ref,1)
                self.ax4 = self.X_ref
                self.ay4 = self.Y_ref
          elif (goal.data == "silo5"):
                self.X_start = [self.current_x,self.current_y,self.current_yaw]
                path9, _ = self.calc_4points_bezier_path(self.X_start, self.X_end8, 4.0, 50)
                self.goal9 = np.vstack([path9[:, 0], path9[:, 1], np.append(np.arctan2(np.diff(path9[:, 1]), np.diff(path9[:, 0])), -1.57)])
                self.goal_states = np.hstack([self.goal9])
                self.X_ref = self.goal_states[0, :]
                self.Y_ref = self.goal_states[1, :]
                self.Yaw_ref = self.smooth_yaw(self.goal_states[2, :])
                self.target_ind = self.calc_index_trajectory(self.current_states[0],self.current_states[1],self.X_ref,self.Y_ref,1)
                self.ax5 = self.X_ref
                self.ay5 = self.Y_ref
          elif (goal.data == "ball"):
                self.X_start = [self.current_x,self.current_y,self.current_yaw]
                path4, _ = self.calc_4points_bezier_path(self.X_start, self.X_end3, 3.0, 50)
                self.goal4 = np.vstack([path4[:, 0], path4[:, 1], np.linspace(1.57,-1.57,50)])
                self.goal_states = np.hstack([self.goal4])
                self.X_ref = self.goal_states[0, :]
                self.Y_ref = self.goal_states[1, :]
                self.Yaw_ref = self.smooth_yaw(self.goal_states[2, :])
                self.target_ind = self.calc_index_trajectory(self.current_states[0],self.current_states[1],self.X_ref,self.Y_ref,1)
                self.ax6 = self.X_ref
                self.ay6 = self.Y_ref
                
     def callback_timer(self):
        if (self.game == "silo1" and self.state == 0):
            self.X_ref, self.Y_ref, self.Yaw_ref, ck = self.silo1(0.1)
            self.Yaw_ref = self.smooth_yaw(self.Yaw_ref)
            self.target_ind = self.calc_index_trajectory(self.current_states[0],self.current_states[1],self.X_ref,self.Y_ref,1)
            self.goal_states = np.vstack([self.X_ref,self.Y_ref,self.Yaw_ref])
            self.ax = self.X_ref
            self.ay = self.Y_ref
            self.state = 1
        # elif (self.state == 1 and self.current_states[0]>=0.35):
        #     self.X_ref, self.Y_ref, self.Yaw_ref, ck = self.silo2(0.1)
        #     self.Yaw_ref = self.smooth_yaw(self.Yaw_ref)
        #     self.target_ind = self.calc_index_trajectory(self.current_states[0],self.current_states[1],self.X_ref,self.Y_ref,1)
        #     self.goal_states = np.vstack([self.X_ref,self.Y_ref,self.Yaw_ref])
        #     self.ax = self.X_ref
        #     self.ay = self.Y_ref
        #     self.state = 2
        if (self.mode_status == "manual"):
            V = self.omni.inverse_kinematic(self.Vx, self.Vy, self.Omega,"numpy")
            self.V = [float(V[0]), float(V[1]), float(V[2]), float(V[3])]
            self.get_logger().info('Velocity of robot: [%f, %f, %f, %f]' % (V[0],V[1],V[2],V[3]))
        elif (self.mode_status == "auto"):
            if (np.linalg.norm(self.goal_states[:,-1]-self.next_trajectories[:,1],2) > 0.1):
                  self.norm_cond += 0.06
                  self.args['lbx'][3*(self.N+1):] = -self.speed_up(self.norm_cond)
                  self.args['ubx'][3*(self.N+1):] =  self.speed_up(self.norm_cond)
            elif (np.linalg.norm(self.goal_states[:,-1]-self.next_trajectories[:,1], 2) < 0.01):
                  self.norm_cond = 0.0

            else:
                  self.args['lbx'][3*(self.N+1):] = self.lowU
                  self.args['ubx'][3*(self.N+1):] = self.highU
                  self.norm_cond = 0.0
            self.args['p'] = np.concatenate([
                self.next_trajectories.T.reshape(-1,1),
                self.next_controls.T.reshape(-1,1)
            ])
            self.args['x0'] = np.concatenate([
                self.states.T.reshape(-1,1),
                self.controls.T.reshape(-1,1)
            ])
            sol = self.solver(
                x0= self.args['x0'],
                p = self.args['p'],
                lbx = self.args['lbx'],
                ubx = self.args['ubx'],
                lbg = self.args['lbg'],
                ubg = self.args['ubg'],
            )

            sol_x = ca.reshape(sol['x'][:3*(self.N+1)], 3, self.N+1)
            sol_u = ca.reshape(sol['x'][3*(self.N+1):], 4, self.N)

            self.u1 = sol_u.full()[0, 0]
            self.u2 = sol_u.full()[1, 0]
            self.u3 = sol_u.full()[2, 0]
            self.u4 = sol_u.full()[3, 0]

            theta = sol_x.full()[2, 0]

            self.target_ind = self.calc_index_trajectory(self.current_states[0], self.current_states[1], self.X_ref, self.Y_ref, self.target_ind)

            self.travel = 2.0

            self.prev_dind = self.next_dind
            for i in range (self.N):
                    for_vec = self.omni.forward_kinematic(self.u1,self.u2,self.u3,self.u4,theta,"numpy")
                    v = np.sqrt(for_vec[0]**2+for_vec[1]**2)
                    self.travel += abs(v)*self.T
                    dind = int(round(self.travel/1.0))
                    pred_index = self.target_ind + self.index

                    self.next_trajectories[0, 0] = self.current_states[0]
                    self.next_trajectories[1, 0] = self.current_states[1]
                    self.next_trajectories[2, 0] = self.current_states[2]
                  #   if pred_index >= self.goal_states.shape[1]:
                  #           pred_index = self.goal_states.shape[1]-1
                    
                    if (self.target_ind + self.index) < len(self.X_ref):
                            self.next_trajectories[0, i+1] = self.X_ref[pred_index]
                            self.next_trajectories[1, i+1] = self.Y_ref[pred_index]
                            self.next_trajectories[2, i+1] = self.Yaw_ref[pred_index]
                    else:
                            self.next_trajectories[0, i+1] = self.X_ref[len(self.X_ref)-1]
                            self.next_trajectories[1, i+1] = self.Y_ref[len(self.X_ref)-1]
                            self.next_trajectories[2, i+1] = self.Yaw_ref[len(self.X_ref)-1]

                    self.next_controls = np.tile(np.array([self.highU, self.highU, self.highU, self.highU]).reshape(4, 1), self.N) 
                    # self.next_controls = np.tile(np.array([20, 20, 20, 20]).reshape(4, 1), self.N) 
            # con_msg = Float32MultiArray()
            self.V = [float(self.u1), float(self.u2), float(self.u3), float(self.u4)]
            # self.pub_velocity.publish(con_msg)
            self.next_dind = dind
            if (self.next_dind - self.prev_dind) >= 2 :
                    self.index += 3
            if pred_index >= len(self.X_ref):
                pred_index = len(self.X_ref)-1
            
            self.current_states = np.array([self.current_x,self.current_y,self.current_yaw])
            # self.current_controls = [(self.u1), float(self.u2), (self.u3), float(self.u4)]
            self.current_controls = np.array([self.u1, self.u2, self.u3, self.u4])
            # self.feedback_state = np.array([self.current_x,self.current_y,self.current_yaw])
            # x_next = self.feedback_state + self.omni.forward_kinematic(self.current_controls[0], self.current_controls[1], self.current_controls[2], self.current_controls[3], theta,"numpy") * self.T
            # self.current_states = x_next
            # self.current_states[2] = self.current_yaw
            self.get_logger().info('Position of robot: [%f, %f, %f]' % (self.current_states[0],self.current_states[1],self.current_states[2]))
            self.states = np.tile(self.current_states.reshape(3, 1), self.N+1)
            self.controls = np.tile(self.current_controls.reshape(4, 1), self.N)
            # self.get_logger().info('Velocity 4 wheel: [%f, %f, %f, %f]' % (self.u1,self.u2,self.u3,self.u4))
            
     def velocity_transmit(self):
            con_msg = Float32MultiArray()
            con_msg.data = [self.u1, float(self.u2), float(self.u3), float(self.u4)]
            self.pub_velocity.publish(con_msg)
            self.get_logger().info('Velocity 4 wheel: [%f, %f, %f, %f]' % (self.u1,self.u2,self.u3,self.u4))
            
def main(args=None):
    rclpy.init(args=args)
    mpc_sim = Simulation_MPC()
    rclpy.spin(mpc_sim)
    mpc_sim.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
               
               