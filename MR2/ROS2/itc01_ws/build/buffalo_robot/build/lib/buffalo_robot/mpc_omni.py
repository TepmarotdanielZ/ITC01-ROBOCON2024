import numpy as np
import casadi as ca
from buffalo_robot.omni_buffalo import Omni_model

class MPC_solver():
    def __init__(self,lowX,highX,lowU,highU,Q,R,N,T):

        ###*** Lower and upper of state and control
        self.lowX = lowX
        self.highX = highX
        self.lowU = lowU
        self.highU = highU

        #Tunnig Matrix of cost funtion
        self.Q = Q
        self.R = R

        #Number of predicted and time
        self.N = N
        self.T = T

        # setup Omni wheel
        r = 0.06
        lx = 0.45/2
        ly = 0.45/2
        self.omni = Omni_model(r,lx,ly)


    def setup_MPC(self):
        # state
        x = ca.SX.sym('x')
        y = ca.SX.sym('y')
        theta = ca.SX.sym('theta')
        state = ca.vertcat(x,y,theta)
        n_state = state.numel()

        # control
        u1 = ca.SX.sym('u1')
        u2 = ca.SX.sym('u2')
        u3 = ca.SX.sym('u3')
        u4 = ca.SX.sym('u4')
        control = ca.vertcat(u1,u2,u3,u4)
        n_control = control.numel()

        X = ca.SX.sym('X',n_state,self.N+1)
        X_ref = ca.SX.sym('X_ref',n_state,self.N+1)
        U = ca.SX.sym('U',n_control,self.N)
        U_ref = ca.SX.sym('U_ref',n_control,self.N)

        # Tuning Matrix
        Q = np.diag(self.Q)
        R = np.diag(self.R)

         # Cost
        cost_fn = 0.0
        g = X[:, 0] - X_ref[:, 0]
        rhs = self.omni.forward_kinematic(u1,u2,u3,u4,theta,"sym")
        f = ca.Function('f',[state,control],[rhs])
        # Euler
        # for k in range(self.N):
        #     st_err = X[:, k] - X_ref[:, k]
        #     con_err = U[:, k] - U_ref[:, k]
        #     cost_fn = cost_fn + st_err.T@Q@st_err + con_err.T@R@con_err
        #     st_next = X[:, k+1]
        #     st_euler = X[:, k] + self.T*f(X[:,k], U[:,k])
        #     g = ca.vertcat(g,st_next-st_euler)

        #RUNG KATA
        for k in range(self.N):
            st_err = X[:, k] - X_ref[:, k]
            con_err = U[:, k] - U_ref[:, k]
            cost_fn = cost_fn + st_err.T @ Q @ st_err + con_err.T @ R @ con_err
            st_next = X[:, k+1]
            k1 = f(X[:,k],U[:,k])
            k2 = f((X[:,k]+(self.T/2)*k1),U[:,k])
            k3 = f((X[:,k]+(self.T/2)*k2),U[:,k])
            k4 = f((X[:,k]+(self.T)*k3),U[:,k])
            st_next_RK4 = X[:,k] + (self.T / 6) * (k1 + 2 * k2 + 2 * k3 + k4)
            g = ca.vertcat(g, st_next - st_next_RK4)
        # cost function J = sigma (X^T@Q@X + U^T@R@U)
        cost_fn = cost_fn + (X[:, self.N]-X_ref[:, self.N]).T@Q@(X[:, self.N]-X_ref[:, self.N])

        # OPimal variable
        opt_var = ca.vertcat(ca.reshape(X,-1, 1),
                             ca.reshape(U,-1, 1)
        )
        # Optimal parameter
        opt_par = ca.vertcat(
            ca.reshape(X_ref, -1, 1),
            ca.reshape(U_ref, -1, 1)
        )

        #Nonilinear program
        nlp_prob = {
            'f': cost_fn,
            'x': opt_var,
            'p': opt_par,
            'g': g
        }

        nlp_opts = {
            'ipopt.max_iter': 2000,
            'ipopt.print_level': 0,
            'ipopt.acceptable_tol': 1e-6,
            'ipopt.acceptable_obj_change_tol': 1e-4,
            'print_time': 0
        }

        solver = ca.nlpsol('solver','ipopt', nlp_prob, nlp_opts)

        lbx = ca.DM.zeros((n_state*(self.N+1) + n_control*self.N,1))
        ubx = ca.DM.zeros((n_state*(self.N+1) + n_control*self.N,1))

        lbx[0: n_state*(self.N+1): n_state] = self.lowX[0]
        lbx[1: n_state*(self.N+1): n_state] = self.lowX[1]
        lbx[2: n_state*(self.N+1): n_state] = self.lowX[2]

        ubx[0: n_state*(self.N+1): n_state] = self.highX[0]
        ubx[1: n_state*(self.N+1): n_state] = self.highX[1]
        ubx[2: n_state*(self.N+1): n_state] = self.highX[2]

        lbx[n_state*(self.N+1)  : n_state*(self.N+1)+n_control*self.N: 4] = self.lowU
        lbx[n_state*(self.N+1)+1: n_state*(self.N+1)+n_control*self.N: 4] = self.lowU
        lbx[n_state*(self.N+1)+2: n_state*(self.N+1)+n_control*self.N: 4] = self.lowU
        lbx[n_state*(self.N+1)+3: n_state*(self.N+1)+n_control*self.N: 4] = self.lowU

        ubx[n_state*(self.N+1)  : n_state*(self.N+1)+n_control*self.N: 4] = self.highU
        ubx[n_state*(self.N+1)+1: n_state*(self.N+1)+n_control*self.N: 4] = self.highU
        ubx[n_state*(self.N+1)+2: n_state*(self.N+1)+n_control*self.N: 4] = self.highU
        ubx[n_state*(self.N+1)+3: n_state*(self.N+1)+n_control*self.N: 4] = self.highU

        args = {
            'lbg': ca.DM.zeros((n_state*(self.N+1), 1)),
            'ubg': ca.DM.zeros((n_state*(self.N+1), 1)),
            'lbx': lbx,
            'ubx': ubx
        }

        return f, solver, args






