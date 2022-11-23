
import os
import sys
import shutil
import casadi as cs
import numpy as np
from copy import copy
import matplotlib
import matplotlib.pyplot as plt
from acados_template import AcadosOcp, AcadosOcpSolver, AcadosModel
from math import sqrt




class px4_quad:
    def __init__(self):
        # Quadrotor intrinsic parameters
        self.J = np.array([.03, .03, .06])  # N m s^2 = kg m^2
        self.mass = 1.5  # kg

        # Length of motor to CoG segment
        self.length = 0.47 / 2  # m
        self.max_thrust = 20
        self.g = np.array([[0], [0], [9.81]])  # m s^-2
        h = np.cos(np.pi / 4) * self.length
        self.x_f = np.array([h, -h, -h, h])
        self.y_f = np.array([-h, -h, h, h])
        self.c = 0.013  # m   (z torque generated by each motor)
        self.z_l_tau = np.array([-self.c, self.c, -self.c, self.c])

        # Input constraints
        self.max_input_value = 1  # Motors at full thrust
        self.min_input_value = -1  # Motors turned off
        self.min_u = self.min_input_value
        self.max_u = self.max_input_value



def linear_quad_model():

    # Declare model variables
    roll = cs.MX.sym('roll')  # position
    pitch = cs.MX.sym('pitch')
    yaw = cs.MX.sym('yaw')

    x = cs.MX.sym('x')
    y = cs.MX.sym('y')
    z = cs.MX.sym('z')

    p = cs.MX.sym('p')
    q = cs.MX.sym('q')
    r = cs.MX.sym('r')

    vx = cs.MX.sym('vx')
    vy = cs.MX.sym('vy')
    vz = cs.MX.sym('vz')
    # Full state vector (13-dimensional)
    x = cs.vertcat(x,y,z,roll,pitch,yaw,vx,vy,vz,p,q,r)
    state_dim = 12

    # Control input vector
    u1 = cs.MX.sym('u1')
    u2 = cs.MX.sym('u2')
    u3 = cs.MX.sym('u3')
    u4 = cs.MX.sym('u4')
    u = cs.vertcat(u1, u2, u3, u4)

    my_quad = px4_quad()
    # p_dynamics
    pos_dynamics = cs.vertcat(vx,vy,vz)

    #q_dynamics
    angle_dynamics = cs.vertcat(
        p+r*pitch+q*roll*pitch,
        q-r*roll,
        r+q*roll)

    # v_dynamics
    g = 9.81
    ft = (u1 + u2 + u3 + u4)*my_quad.max_thrust
    taux = (u3 - u1)*my_quad.max_thrust*my_quad.length
    tauy = (u4 - u2)*my_quad.max_thrust*my_quad.length
    tauz = (u2 + u4 - u1 - u3)*my_quad.max_thrust*my_quad.length
    v_dynamics = cs.vertcat(
        r*vy-q*vz-g*pitch,
        p*vz-r*vx+g*roll,
        q*vx-p*vy + g - ft/my_quad.mass)
    #w_dynamics 
    w_dynamics = cs.vertcat(
            (my_quad.J[1] - my_quad.J[2])/my_quad.J[0] * r * q + taux/my_quad.J[0],
            (my_quad.J[2] - my_quad.J[0])/my_quad.J[1] * r * p + tauy/my_quad.J[1],
            (my_quad.J[0] - my_quad.J[1])/my_quad.J[2] * p * q + tauz/my_quad.J[2])

    """
    x_dot = cs.vertcat(p_dynamics, q_dynamics, v_dynamics, w_dynamics)
    quad_dynamic = cs.Function('x_dot', [x[:13],u], [x_dot], ['x', 'u'], ['x_dot'])
    normails = quad_dynamic(x=x, u=u)['x_dot']
    x_dot = cs.MX.sym('x_dot', normails.shape)
    f_impl = x_dot - normails
    """
    pdot = cs.MX.sym('pdot', 3)  # position
    qdot = cs.MX.sym('adot', 3)  # angle roll pitch yaw
    vdot = cs.MX.sym('vdot', 3)  # velocity
    rdot = cs.MX.sym('rdot', 3)  # angle rate
    xdot = cs.vertcat(pdot, qdot, vdot, rdot)

    normails = cs.vertcat(pos_dynamics, angle_dynamics, v_dynamics, w_dynamics)
    f_impl = xdot - normails

    model_name = 'px4_quad_linear_model'

    # Dynamics model
    model = AcadosModel()
    model.f_expl_expr = normails
    model.f_impl_expr = f_impl
    model.x = x
    model.xdot = xdot
    model.u = u
    model.p = []
    model.name = model_name

    return model

def DT_linear_model(dT):
    model = linear_quad_model()
    x = model.x
    u = model.u

    ode = cs.Function('ode',[x, u], [model.f_expl_expr])

    # set up Rk4
    k1 = ode(x,       u)
    k2 = ode(x+dT/2*k1,u)
    k3 = ode(x+dT/2*k2,u)
    k4 = ode(x+dT*k3,  u)
    xf = x + dT/6 * (k1 + 2*k2 + 2*k3 + k4)

    model.disc_dyn_expr = xf
    return model
    
def acados_settinngs(solver_options = None,t_horizon = 1,N = 20,dT = 1/10):
    
    my_quad = px4_quad()

    acados_models = DT_linear_model(dT)
    
    q_cost = np.array([12, 12, 12, 2, 2, 2, 0.5, 0.5, 0.5, 1, 1, 1])
    r_cost = np.array([0.5, 0.5, 0.5, 0.5])
     
    

    nx = acados_models.x.size()[0]
    nu = acados_models.u.size()[0]
    ny = nx + nu
    n_param = acados_models.p.size()[0] if isinstance(acados_models.p, cs.MX) else 0
    ocp = AcadosOcp()
    ocp.model = acados_models
    ocp.dims.N = N
    ocp.solver_options.tf = t_horizon

    # Initialize parameters

    ocp.dims.np = n_param
    ocp.parameter_values = np.zeros(n_param)

    ocp.cost.cost_type = 'LINEAR_LS'
    ocp.cost.cost_type_e = 'LINEAR_LS'
    ocp.cost.W = np.diag(np.concatenate((q_cost, r_cost)))
    ocp.cost.W_e = np.diag(q_cost)
    terminal_cost = 0 if solver_options is None or not solver_options["terminal_cost"] else 1
    ocp.cost.W_e *= 1

    ocp.cost.Vx = np.zeros((ny, nx))
    ocp.cost.Vx[:nx, :nx] = np.eye(nx)
    ocp.cost.Vu = np.zeros((ny, nu))
    ocp.cost.Vu[-4:, -4:] = np.eye(nu)

    ocp.cost.Vx_e = np.eye(nx)

    # Initial reference trajectory (will be overwritten)
    x_ref = np.zeros(nx)
    ocp.cost.yref = np.concatenate((x_ref, np.array([0.0, 0.0, 0.0, 0.0])))
    ocp.cost.yref_e = x_ref

    ocp.constraints.x0 = x_ref

    # Set constraints
    ocp.constraints.lbu = np.array([my_quad.min_u] * 4)
    ocp.constraints.ubu = np.array([my_quad.max_u] * 4)
    ocp.constraints.idxbu = np.array([0, 1, 2, 3])
    # Solver options
    ocp.solver_options.qp_solver = 'FULL_CONDENSING_HPIPM'
    ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
    ocp.solver_options.integrator_type = 'DISCRETE'
    ocp.solver_options.print_level = 0
    ocp.solver_options.nlp_solver_type = 'SQP_RTI'
    acados_solver = AcadosOcpSolver(ocp, json_file='acados_ocp.json')

    return acados_solver



def run_solver(N,model,acados_solver,initial_state,ref):
    
    u_target = np.zeros((N+1,4))
    ref = np.concatenate((ref, u_target),axis = 1)
    for j in range(N):
        acados_solver.set(j, "yref", ref[j])
    acados_solver.set(N, "yref", ref[N][:-4])
    
    # Set initial state.
    x_init = initial_state
    x_init = np.stack(x_init)
    # Set initial condition, equality constraint
    acados_solver.set(0, 'lbx', x_init)
    acados_solver.set(0, 'ubx', x_init)

    # Solve OCP
    acados_solver.solve()
    # get vx vy vz 
    x_next = acados_solver.get(1, "x")
    vx_next = x_next[6]
    vy_next = x_next[7]
    vz_next = x_next[8]
    p_next = x_next[9]
    q_next = x_next[10]
    r_next = x_next[11]	

    return vx_next,vy_next,vz_next,p_next,q_next,r_next






