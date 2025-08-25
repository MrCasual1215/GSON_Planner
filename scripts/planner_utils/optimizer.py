import datetime
import time

import casadi as ca
import numpy as np

class NmpcDcbfOptimizerParam:
    def __init__(self, 
            horizon=20,
            horizon_dcbf=5,
            dis_Q = 100,
            dis_T = 10,
            mat_R = np.diag([10.0, 10.0]),
            mat_Q = np.diag([10.0, 10.0]),
            mat_Rold = np.diag([0, 0.]) * 0.0,
            mat_dR = np.diag([1.0, 1.0]) * 0.0,
            gamma = 0.8,
            pomega = 10.0,
            terminal_weight = 10.0,
            safe_dist = 0.5,
            ROI = 2,
        ):
        self.horizon = horizon
        self.horizon_dcbf = horizon_dcbf
        self.dis_Q = dis_Q
        self.dis_T = dis_T
        self.mat_R = mat_R
        self.mat_Q = mat_Q
        self.mat_Rold = mat_Rold
        self.mat_dR = mat_dR
        self.gamma = gamma
        self.pomega = pomega
        self.terminal_weight = terminal_weight
        self.safe_dist = safe_dist
        self.ROI = ROI



class NmpcDbcfOptimizer:
    def __init__(self, variables: dict, costs: dict, timestep: float, param):
        self.opti = None
        self.variables = variables
        self.costs = costs
        self.dynamics_opt = self._dynamics_opt(timestep)
        self.solver_times = []
        self.setup_times = []
        self.param = param
        self.xs = np.zeros((3, param.horizon + 1))
        self.us = np.zeros((2, param.horizon))


    def _dynamics_opt(self, timestep):
        """Return updated state in a form of `ca.SX`"""
        x_symbol = ca.SX.sym("x", 3)
        u_symbol = ca.SX.sym("u", 2)

        xd_symbol_next = x_symbol[0] + u_symbol[0]*ca.cos(x_symbol[2]) * timestep
        yd_symbol_next = x_symbol[1] + u_symbol[0]*ca.sin(x_symbol[2]) * timestep
        theta_symbol_next = x_symbol[2] + u_symbol[1] * timestep

        state_symbol_next = ca.vertcat(xd_symbol_next, yd_symbol_next, theta_symbol_next)
        return ca.Function("dynamics", [x_symbol, u_symbol], [state_symbol_next])


    def set_state(self, state):
        self.state = state


    def initialize_variables(self, param):
        self.variables["x"] = self.opti.variable(3, param.horizon + 1)
        self.variables["u"] = self.opti.variable(2, param.horizon)

        # self.variables["slack_variables"] = self.opti.variable(1)

    def add_initial_condition_constraint(self):
        self.opti.subject_to(self.variables["x"][:, 0] == self.state)

    def add_input_constraint(self, param):
        # TODO: wrap params
        v_max, omegamax = 0.3, 0.4
        for i in range(param.horizon):
            # input constraints
            self.opti.subject_to(self.variables["u"][0, i] <= v_max)
            self.opti.subject_to(-v_max <= self.variables["u"][0, i])
            self.opti.subject_to(self.variables["u"][1, i] <= omegamax)
            self.opti.subject_to(-omegamax <= self.variables["u"][1, i])


    def add_input_derivative_constraint(self, param):
        # TODO: Remove this hardcoded function with timestep
        a_max, omegadot_max = 0.3, 0.4
        for i in range(param.horizon - 1):
            # input constraints
            self.opti.subject_to(self.variables["u"][0, i + 1] - self.variables["u"][0, i] <= a_max)
            self.opti.subject_to(self.variables["u"][0, i + 1] - self.variables["u"][0, i] >= -a_max)
            self.opti.subject_to(self.variables["u"][1, i + 1] - self.variables["u"][1, i] <= omegadot_max)
            self.opti.subject_to(self.variables["u"][1, i + 1] - self.variables["u"][1, i] >= -omegadot_max)

        self.opti.subject_to(self.variables["u"][0, 0] - self.us[0,0] <= a_max)
        self.opti.subject_to(self.variables["u"][0, 0] - self.us[0,0] >= -a_max)
        self.opti.subject_to(self.variables["u"][1, 0] - self.us[1,0] <= omegadot_max)
        self.opti.subject_to(self.variables["u"][1, 0] - self.us[1,0] >= -omegadot_max)

    def add_dynamics_constraint(self, param):
        for i in range(param.horizon):
            self.opti.subject_to(
                self.variables["x"][:, i + 1] == self.dynamics_opt(self.variables["x"][:, i], self.variables["u"][:, i])
            )

    def add_input_stage_cost(self, param):
        self.costs["input_stage"] = 0
        for i in range(param.horizon):
            self.costs["input_stage"] += ca.mtimes(
                self.variables["u"][:, i].T, ca.mtimes(param.mat_R, self.variables["u"][:, i])
            )

    def add_prev_input_cost(self, param):
        self.costs["prev_input"] = 0
        self.costs["prev_input"] += ca.mtimes(
            (self.variables["u"][:, 0] - self.us[:,0]).T,
            ca.mtimes(param.mat_Rold, (self.variables["u"][:, 0] - self.us[:,0])),
        )

    def add_input_smoothness_cost(self, param):
        self.costs["input_smoothness"] = 0
        for i in range(param.horizon - 1):
            self.costs["input_smoothness"] += ca.mtimes(
                (self.variables["u"][:, i + 1] - self.variables["u"][:, i]).T,
                ca.mtimes(param.mat_dR, (self.variables["u"][:, i + 1] - self.variables["u"][:, i])),
            )


    def add_obstacle_avoidance_constraint(self, param, obstacles):
        self.costs["decay_rate_relaxing"] = 0
        # TODO: wrap params
        # TODO: move safe dist inside attribute `system`
        
        for obs_points in obstacles:
            if len(obs_points) < 1:
                continue
            else:
                self.add_point_to_point_constraint(param, obs_points, param.safe_dist, param.ROI)
 

    def add_reference_trajectory_tracking_cost(self, param, reference_trajectory):
        self.costs["reference_trajectory_tracking"] = 0
        for i in range(param.horizon - 1):
            x_diff = self.variables["x"][:2, i] - reference_trajectory[i, :].T
            self.costs["reference_trajectory_tracking"] += ca.mtimes(x_diff.T, ca.mtimes(param.mat_Q, x_diff))
        x_diff = self.variables["x"][:2, -1] - reference_trajectory[-1, :]
        self.costs["reference_trajectory_tracking"] += param.terminal_weight * ca.mtimes(x_diff.T, ca.mtimes(param.mat_Q, x_diff))


    def add_point_to_point_constraint(self, param, obs_points, safe_dist, ROI):
        # get current value of cbf
        # print(obs_points)
        cbf_curr = np.linalg.norm(self.state[0:2].T - obs_points[0])

        # filter obstacle if it's still far away
        if cbf_curr > ROI:
            return
        
        # print("Adding obstacle avoidance constraint for points:", obs_points[0])
        
        h = lambda x_, obs: (x_[0] - obs[0]) ** 2 + (x_[1] - obs[1]) ** 2 - (safe_dist)**2
        omega = self.opti.variable(param.horizon_dcbf-1, 1)

        for i in range(param.horizon_dcbf - 1):
            self.opti.subject_to(h(self.variables["x"][0:2, i + 1], obs_points[i+1]) >= omega[i] * param.gamma ** (i + 1) * h(self.variables["x"][0:2, i], obs_points[i]) ) 
            self.opti.subject_to(omega[i] >= 0)
            self.costs["decay_rate_relaxing"] +=  param.pomega * (omega[i] - 1) ** 2
            # warm start
            self.opti.set_initial(omega[i], 1)


    def add_warm_start(self, param):
        # TODO: wrap params
        # x_ws, u_ws = self.state, np.zeros((2, 1))
        self.opti.set_initial(self.variables["x"][:,0:self.param.horizon-2], self.xs[:,1:self.param.horizon-1])
        self.opti.set_initial(self.variables["u"][:,0:self.param.horizon-2], self.us[:,1:self.param.horizon-1])
        self.opti.set_initial(self.variables["x"][:,-1], self.xs[:,-1])
        self.opti.set_initial(self.variables["u"][:,-1], self.us[:,-1])


    def setup(self, state, local_reference, obstacles):
        param = self.param
        start_timer = datetime.datetime.now()
        self.set_state(state)
        self.opti = ca.Opti()
        self.initialize_variables(param)
        self.add_initial_condition_constraint()
        self.add_input_constraint(param)
        self.add_input_derivative_constraint(param)
        self.add_dynamics_constraint(param)
        self.add_reference_trajectory_tracking_cost(param, local_reference)
        self.add_input_stage_cost(param)
        self.add_prev_input_cost(param)
        self.add_input_smoothness_cost(param)
        self.add_obstacle_avoidance_constraint(param, obstacles)
        self.add_warm_start(param)
        end_timer = datetime.datetime.now()
        self.setup_times.append((end_timer - start_timer).total_seconds())
        # print("setup time: ", (end_timer - start_timer).total_seconds())

    def solve_nlp(self):
        cost = 0
        for cost_name in self.costs:
            cost += self.costs[cost_name]
        self.opti.minimize(cost)
        option = {"verbose": False, "ipopt.print_level": 0, "print_time": 0, "expand": True}
        start_timer = datetime.datetime.now()
        self.opti.solver("ipopt", option)
        try:
            opt_sol = self.opti.solve()
            control_input = opt_sol.value(self.variables["u"][:, 0])
        except:
            print("solver failed")
            control_input = self.opti.debug.value(self.variables["u"][:, 0])
        
        # print(opt_sol.value(self.variables["x"]))
        self.xs = opt_sol.value(self.variables["x"])
        self.us = opt_sol.value(self.variables["u"])


        end_timer = datetime.datetime.now()
        delta_timer = end_timer - start_timer
        self.solver_times.append(delta_timer.total_seconds())
        # print("solver time: ", delta_timer.total_seconds())
        return control_input
