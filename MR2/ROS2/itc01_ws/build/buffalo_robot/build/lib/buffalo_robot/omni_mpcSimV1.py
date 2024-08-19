import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray,Float32
import casadi as ca
import numpy as np
import matplotlib.pyplot as plt
import math
import time
from buffalo_robot.mpc_omni import MPC_solver
from buffalo_robot.Omni_kinematic import OmniModel
from buffalo_robot.bezier_path import calc_bezier_path

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
          self.timer_callback = self.create_timer(timer,self.callback_timer)
          self.goal_sub = self.create_subscription(String,'/goal', self.subscript_goal,10)
          self.omni = OmniModel()

          #Setup MPC
          self.lowX = [-15, -15, -3.14]
          self.highX = [15, 15, 3.14]
          self.lowU = -40.0
          self.highU = 40.0
          Q = [900, 900, 700]
          R = [0.01, 0.01, 0.01, 0.01]
          self.N = 30
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
          self.speed_up = lambda t: 40*(1-np.exp(-2*t))
          self.u1 = 0.0
          self.u2 = 0.0
          self.u3 = 0.0
          self.u4 = 0.0
          self.current_states = np.array([0.0, 0.0, 0.0], dtype=np.float64)
          self.current_controls = np.array([0, 0, 0, 0], dtype=np.float64)

          # Path_Success:(goal1,goal2,goal3)
          # Path_Retry: (goal4,goal2,gaol3)
          # Path_Silo1: (goal5), Silo2:(goal6), Silo3:(goal7), Silo4:(goal8), Silo5:(goal9)
          # Path_Ball: (gaol10)
          self.X_start = [0.0, 0.0, 0.0]
          self.X_End = [0.0, 0.0, 0.0]
          self.X_end  = [3.0, 0.0, 0.0]    #  Test in lab
          self.X_end1 = [3.50, -2.5, -0.0]  # 
          self.X_end2 = [3.0, 0.0, -1.57]
      #     self.X_end3 = [10.0, -1.5, -1.57]  #Ball
      #     self.X_end  = [5.0, 0.0, 0.0]    #  Retry_start
      #     self.X_end1 = [6.0, -3.2, -1.57]  # 
        #   self.X_end2 = [8.0, -3.5, 1.57]
          self.X_end3 = [10.0, -1.5, -1.57]  #Ball
          self.X_end4 = [8.1, -4.28, -1.57]  #Silo1
          self.X_end5 = [8.9, -4.28, -1.57]  # Silo2
          self.X_end6 = [9.65, -4.28, -1.57] # Silo3
          self.X_end7 = [10.4, -4.28, -1.57] # Silo4
          self.X_end8 = [11.15, -4.28, -1.57] # Silo5
          self.goal0 = np.array([[0,0,0],[0,0,0],[0,0,0]])
          self.goal1 = [0]
          self.goal2 = [0]
          self.goal3 = [0]
          self.goal4 = [0]
          self.goal5 = [0]
          self.goal6 = [0]
          self.goal7 = [0]
          self.goal8 = [0]
          self.goal_states = np.hstack([self.goal0])

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
     def subscript_goal(self,goal):
          if (goal.data == "success"):
                self.X_start = [0.0,0.0,0.0]
                path1, _ = self.calc_4points_bezier_path(self.X_start, self.X_end, 2.5, 50)
                path2, _ = self.calc_4points_bezier_path(self.X_end, self.X_end1, 3.0, 50)
            #     path3, _ = self.calc_4points_bezier_path(self.X_end1, self.X_end2, 5.0, 50)
            #     path4, _ = self.calc_4points_bezier_path(self.X_end2, self.X_end3, 6.0, 50)
                self.goal1 = np.vstack([path1[:, 0], path1[:, 1], np.append(np.arctan2(np.diff(path1[:, 1]), np.diff(path1[:, 0])), 0.0)])
                self.goal2 = np.vstack([path2[:, 0], path2[:, 1], np.append(np.arctan2(np.diff(path2[:, 1]), np.diff(path2[:, 0])), 0.0)])
            #     self.goal3 = np.vstack([path3[:, 0], path3[:, 1], np.append(np.arctan2(np.diff(path3[:, 1]), np.diff(path3[:, 0])), 1.57)])
            #     self.goal4 = np.vstack([path4[:, 0], path4[:, 1], np.linspace(1.57,-1.57,50)])
            #     self.goal_states = np.hstack([self.goal1,self.goal2,self.goal3,self.goal4])
                self.goal_states = np.hstack([self.goal1,self.goal2])
                self.X_ref = self.goal_states[0, :]
                self.Y_ref = self.goal_states[1, :]
                self.Yaw_ref = self.smooth_yaw(self.goal_states[2, :])
                self.target_ind = self.calc_index_trajectory(self.current_states[0],self.current_states[1],self.X_ref,self.Y_ref,1)
                self.ax = self.X_ref
                self.ay = self.Y_ref
          elif (goal.data == "retry"):
                self.X_start = [0.0,0.0,0.0]
                path2, _ = self.calc_4points_bezier_path(self.X_start, self.X_end, 2.5, 50)
                # path3, _ = self.calc_4points_bezier_path(self.X_end1, self.X_end2, 4.0, 50)
                # path4, _ = self.calc_4points_bezier_path(self.X_end2, self.X_end3, 6.0, 50)
                self.goal2 = np.vstack([path2[:, 0], path2[:, 1], np.append(np.arctan2(np.diff(path2[:, 1]), np.diff(path2[:, 0])), 0.0)])
                # self.goal3 = np.vstack([path3[:, 0], path3[:, 1], np.append(np.arctan2(np.diff(path3[:, 1]), np.diff(path3[:, 0])), 1.57)])
                # self.goal4 = np.vstack([path4[:, 0], path4[:, 1], np.linspace(1.57,-1.57,50)])
                self.goal_states = np.hstack([self.goal2])
                self.X_ref = self.goal_states[0, :]
                self.Y_ref = self.goal_states[1, :]
                self.Yaw_ref = self.smooth_yaw(self.goal_states[2, :])
                # self.current_states = np.array([5.0, 0.0, 0.0], dtype=np.float64)
                self.target_ind = self.calc_index_trajectory(self.current_states[0],self.current_states[1],self.X_ref,self.Y_ref,1)
                self.ax = self.X_ref
                self.ay = self.Y_ref
          elif (goal.data == "silo1"):
                self.X_start = [self.current_x,self.current_y,self.current_yaw]
                path5, _ = self.calc_4points_bezier_path(self.X_start, self.X_end, 3.0, 50)
                path6, _ = self.calc_4points_bezier_path(self.X_end, self.X_End, 3.0, 50)
                self.goal5 = np.vstack([path5[:, 0], path5[:, 1], np.append(np.arctan2(np.diff(path5[:, 1]), np.diff(path5[:, 0])), -1.57)])
                self.goal6 = np.vstack([path6[:, 0], path6[:, 1], np.append(np.arctan2(np.diff(path6[:, 1]), np.diff(path6[:, 0])), 0.0)])
                self.goal_states = np.hstack([self.goal5,self.goal6])
                self.X_ref = self.goal_states[0, :]
                self.Y_ref = self.goal_states[1, :]
                self.Yaw_ref = self.smooth_yaw(self.goal_states[2, :])
                self.target_ind = self.calc_index_trajectory(self.current_states[0],self.current_states[1],self.X_ref,self.Y_ref,1)
                self.ax1 = self.X_ref
                self.ay1 = self.Y_ref
          elif (goal.data == "silo2"):
                self.X_start = [self.current_x,self.current_y,self.current_yaw]
                path6, _ = self.calc_4points_bezier_path(self.X_start, self.X_end5, 4.0, 50)
                self.goal6 = np.vstack([path6[:, 0], path6[:, 1], np.append(np.arctan2(np.diff(path6[:, 1]), np.diff(path6[:, 0])), -1.57)])
                self.goal_states = np.hstack([self.goal6])
                self.X_ref = self.goal_states[0, :]
                self.Y_ref = self.goal_states[1, :]
                self.Yaw_ref = self.smooth_yaw(self.goal_states[2, :])
                self.target_ind = self.calc_index_trajectory(self.current_states[0],self.current_states[1],self.X_ref,self.Y_ref,1)
                self.ax2 = self.X_ref
                self.ay2 = self.Y_ref
          elif (goal.data == "silo3"):
                self.X_start = [self.current_x,self.current_y,self.current_yaw]
                path7, _ = self.calc_4points_bezier_path(self.X_start, self.X_end6, 4.0, 50)
                self.goal7 = np.vstack([path7[:, 0], path7[:, 1], np.append(np.arctan2(np.diff(path7[:, 1]), np.diff(path7[:, 0])), -1.57)])
                self.goal_states = np.hstack([self.goal7])
                self.X_ref = self.goal_states[0, :]
                self.Y_ref = self.goal_states[1, :]
                self.Yaw_ref = self.smooth_yaw(self.goal_states[2, :])
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

         self.travel = 0.0

         self.prev_dind = self.next_dind
         for i in range (self.N):
                for_vec = self.omni.forward_kinematic(self.u1,self.u2,self.u3,self.u4,theta,"numpy")
                v = np.sqrt(for_vec[0]**2+for_vec[1]**2)
                self.travel += abs(v)*0.1
                dind = int(round(self.travel/1.0))
                pred_index = self.target_ind + self.index

                self.next_trajectories[0, 0] = self.current_states[0]
                self.next_trajectories[1, 0] = self.current_states[1]
                self.next_trajectories[2, 0] = self.current_states[2]
                if pred_index >= self.goal_states.shape[1]:
                        pred_index = self.goal_states.shape[1]-1

                if (self.target_ind + self.index) < len(self.X_ref):
                        self.next_trajectories[0, i+1] = self.X_ref[pred_index]
                        self.next_trajectories[1, i+1] = self.Y_ref[pred_index]
                        self.next_trajectories[2, i+1] = self.Yaw_ref[pred_index]
                else:
                        self.next_trajectories[0, i+1] = self.X_ref[len(self.X_ref)-1]
                        self.next_trajectories[1, i+1] = self.Y_ref[len(self.X_ref)-1]
                        self.next_trajectories[2, i+1] = self.Yaw_ref[len(self.X_ref)-1]

                self.next_controls = np.tile(np.array([self.highU, self.highU, self.highU, self.highU]).reshape(4, 1), self.N) 

         self.next_dind = dind
         if (self.next_dind - self.prev_dind) >= 2 :
                self.index += 1

         x_next = self.current_states + self.omni.forward_kinematic(self.u1, self.u2, self.u3, self.u4, theta,"numpy") * self.T
         self.current_states = x_next
         self.current_x = self.current_states[0]
         self.current_y = self.current_states[1]
         self.current_yaw = self.current_states[2]
         self.current_controls = np.array([self.u1, self.u2, self.u3, self.u4])
         
         self.states = np.tile(self.current_states.reshape(3, 1), self.N+1)
         self.controls = np.tile(self.current_controls.reshape(4, 1), self.N)
         self.get_logger().info('Velocity 4 wheel: [%f, %f, %f, %f]' % (self.u1,self.u2,self.u3,self.u4))
         plt.clf()
         plt.plot(self.ax, self.ay, "b")
         plt.plot(self.ax1, self.ay1, "b")
         plt.plot(self.ax2, self.ay2, "b")
         plt.plot(self.ax3, self.ay3, "b")
         plt.plot(self.ax4, self.ay4, "b")
         plt.plot(self.ax5, self.ay5, "b")
         plt.plot(self.ax6, self.ay6, "b")
         plt.gcf().canvas.mpl_connect('key_release_event',
                        lambda event: [exit(0) if event.key == 'escape' else None])
         plot_arrow(self.current_states[0], self.current_states[1], self.current_states[2])
         plt.plot(self.ax, self.ay, marker="x", color="blue", label="Input Trajectory")
         plt.scatter(sol_x.full()[0, :], sol_x.full()[1, :], marker="*", color="red", label="Predicted value")
         plt.plot(self.current_states[0], self.current_states[1], marker="*", color="black")
         plt.title("Velocity of robot [m/sec]:" + str(round(v,2)))
         plt.axis("equal")
         plt.grid(True)
         plt.legend()
         plt.pause(0.0001)
         self.mpciter += 1

def main(args=None):
    rclpy.init(args=args)
    mpc_sim = Simulation_MPC()
    rclpy.spin(mpc_sim)
    mpc_sim.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
               
               
