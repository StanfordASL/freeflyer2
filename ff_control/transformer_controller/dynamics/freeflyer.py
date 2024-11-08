import os
import sys

root_folder = os.path.abspath(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.append(root_folder)

import numpy as np
import cvxpy as cp
import optimization.ff_scenario as ff
import copy

# -----------------------------------------
# Freeflyer model
class FreeflyerModel:

    N_STATE = ff.N_STATE
    N_ACTION = ff.N_ACTION
    N_CLUSTERS = ff.N_CLUSTERS

    def __init__(self, param=None, verbose=False):
        # Initialization
        self.verbose = verbose
        if param is None:
            self.param = {
                'mass' : ff.mass,
                'J' : ff.inertia,
                'radius' : ff.robot_radius,
                'F_t_M' : ff.F_max_per_thruster,
                'b_t' : ff.thrusters_lever_arm,
                'Lambda' : ff.Lambda,
                'Lambda_inv' : ff.Lambda_inv
            }
        else:
            if ((ff.mass == param['mass']) and (ff.inertia == param['J']) and (ff.robot_radius == param['radius']) and (ff.F_max_per_thruster == param['F_t_M'])
                and (ff.thrusters_lever_arm == param['b_t']) and (ff.Lambda == param['Lambda']).all() and (ff.Lambda_inv == param['Lambda_inv']).all()):
                self.param = copy.deepcopy(param)
            else:
                raise ValueError('The scenario parameter specified in ROS and in ff_scenario.py are not the same!!')
        
        if self.verbose:
            print("Initializing freeflyer class.")

        # Full system dynamics
        self.A = np.array([[0, 0, 0, 1, 0, 0],
                           [0, 0, 0, 0, 1, 0],
                           [0, 0, 0, 0, 0, 1],
                           [0, 0, 0, 0, 0, 0],
                           [0, 0, 0, 0, 0, 0],
                           [0, 0, 0, 0, 0, 0]])
        self.B = np.array([[                   0,                    0,                 0],
                           [                   0,                    0,                 0],
                           [                   0,                    0,                 0],
                           [1/self.param['mass'],                    0,                 0],
                           [                   0, 1/self.param['mass'],                 0],
                           [                   0,                    0, 1/self.param['J']]])
        
        # Linear system for optimization with impulsive DeltaV, DeltaPSI_dot
        self.set_time_discretization(ff.dt)
        self.B_imp = np.array([[0, 0,                                  0],
                               [0, 0,                                  0],
                               [0, 0,                                  0],
                               [1, 0,                                  0],
                               [0, 1,                                  0],
                               [0, 0, self.param['mass']/self.param['J']]])
    
    def f(self, state, action_thrusters):
        if len(action_thrusters) != self.N_CLUSTERS:
            raise TypeError('Use the action of the 4 clusters of thrusters to work with the full dynamics!')
        actions_G = (self.R_GB(state[2]) @ (self.param['Lambda'] @ action_thrusters))
        state_dot = self.A @ state + self.B @ actions_G
        return state_dot
    
    def f_imp(self, state, action_G):
        if len(action_G) != self.N_ACTION:
            raise TypeError('Use the action of in the global reference frame to work with the impulsive dynamics!')
        state_new = self.Ak @ (state + self.B_imp @ action_G)
        return state_new
    
    '''def f_PID(self, state, state_desired):
        control_step_x_opt_step = int(np.round(ff.dt/ff.control_period))
        states = np.zeros((self.N_STATE, control_step_x_opt_step+1))
        states[:,0] = state.copy()
        for i in range(control_step_x_opt_step):
            state_delta = state_desired - states[:,i]
            # wrap angle delta to [-pi, pi]
            state_delta[2] = (state_delta[2] + np.pi) % (2 * np.pi) - np.pi

            u = np.minimum(np.maximum(self.param['Lambda_inv'] @ (self.R_BG(states[2,i]) @ (ff.K @ state_delta)), -self.param['F_t_M']), self.param['F_t_M'])
            #u = self.param['F_t_M']*np.sign(self.param['Lambda_inv'] @ (self.R_BG(states[2,i]) @ (ff.K @ state_delta)))
            states[:,i+1] = states[:,i] + (self.A @ states[:,i] + self.B @ (self.R_GB(states[2,i]) @ (self.param['Lambda'] @ u)))*ff.control_period
        
        return states[:,-1].copy()'''
    
    ################## OPTIMIZATION METHODS ###############
    def initial_guess_line(self, state_init, state_final, final_time=ff.T_nominal):
        tt = np.arange(0,final_time + ff.dt/2, ff.dt)
        state_ref = state_init[:,None] + ((state_final - state_init)[:,None]/final_time)*np.repeat(tt[None,:], self.N_STATE, axis=0)
        action_ref = np.zeros((self.N_ACTION, len(tt)-1))
        return state_ref, action_ref

    def set_time_discretization(self, dt):
        self.Ak = np.eye(self.N_STATE, self.N_STATE) + dt*self.A
        self.Dv_t_M = self.param['F_t_M']*dt/self.param['mass']

    def action_bounding_box_lin(self, psi_ref, action_ref):
        A_bb = 0.5*np.array([-np.cos(psi_ref)*action_ref[0] - np.sin(psi_ref)*action_ref[1],
                             -np.sin(psi_ref)*action_ref[0] + np.cos(psi_ref)*action_ref[1],
                             -np.cos(psi_ref)*action_ref[0] - np.sin(psi_ref)*action_ref[1],
                             -np.sin(psi_ref)*action_ref[0] + np.cos(psi_ref)*action_ref[1]])
        B_bb = np.array([[-np.sin(psi_ref)/2, np.cos(psi_ref)/2,  1/(4*self.param['b_t'])],
                         [ np.cos(psi_ref)/2, np.sin(psi_ref)/2, -1/(4*self.param['b_t'])],
                         [-np.sin(psi_ref)/2, np.cos(psi_ref)/2, -1/(4*self.param['b_t'])],
                         [ np.cos(psi_ref)/2, np.sin(psi_ref)/2,  1/(4*self.param['b_t'])]])

        return A_bb, B_bb

    def ocp_scp(self, state_ref, action_ref, state_init, state_final, obs, trust_region, obs_av=True):
        # Setup SCP problem
        n_time = action_ref.shape[1]
        s = cp.Variable((self.N_STATE,n_time))
        a = cp.Variable((self.N_ACTION,n_time))

        # CONSTRAINTS
        constraints = []

        # Initial, dynamics and final state
        constraints += [s[:,0] == state_init]
        constraints += [s[:,k+1] == self.Ak @ (s[:,k] + self.B_imp @ a[:,k]) for k in range(n_time-1)]
        constraints += [(s[:,-1] + self.B_imp @ a[:,-1]) == state_final]
        # Table extension
        constraints += [s[:2,:] >= ff.start_region['xy_low'][:,None]]
        constraints += [s[:2,:] <= ff.goal_region['xy_up'][:,None]]
        # Trust region and koz and action bounding box
        for k in range(0,n_time):
            # Trust region
            b_soc_k = -state_ref[:,k]
            constraints += [cp.SOC(trust_region, s[:,k] + b_soc_k)]
            # keep-out-zone
            if obs_av:
                for n_obs in range(len(obs['radius'])):
                    c_koz_k = np.transpose(state_ref[:2,k] - obs['position'][n_obs,:]).dot(np.eye(2)/((obs['radius'][n_obs])**2))
                    b_koz_k = np.sqrt(c_koz_k.dot(state_ref[:2,k] - obs['position'][n_obs,:]))
                    constraints += [c_koz_k @ (s[:2,k] - obs['position'][n_obs,:]) >= b_koz_k]
            # action bounding box
            A_bb_k, B_bb_k = self.action_bounding_box_lin(state_ref[2,k], action_ref[:,k])
            constraints += [A_bb_k*(s[2,k] - state_ref[2,k]) + B_bb_k@a[:,k] >= -self.Dv_t_M]
            constraints += [A_bb_k*(s[2,k] - state_ref[2,k]) + B_bb_k@a[:,k] <= self.Dv_t_M]

        # Cost function
        cost = cp.sum(cp.norm(a, 1, axis=0))

        # Problem formulation
        prob = cp.Problem(cp.Minimize(cost), constraints)

        prob.solve(solver=cp.ECOS, verbose=False)
        if prob.status == 'infeasible':
            print("[solve]: Problem infeasible. [obstacle avoidance]:", obs_av)
            s_opt = None
            a_opt = None
            J = None
        else:
            s_opt = s.value
            a_opt = a.value
            s_opt = np.vstack((s_opt.T, s_opt[:,-1] + self.B_imp @ a_opt[:,-1])).T
            J = prob.value

        return s_opt, a_opt, J, prob.status

    ################## STATIC METHODS ######################
    @staticmethod
    def R_GB(psi):
        try:
            R_GB = np.zeros((len(psi),3,3))
            cos_psi = np.cos(psi)
            sin_psi = np.sin(psi)
            R_GB[:,0,0] = cos_psi
            R_GB[:,1,1] = cos_psi
            R_GB[:,0,1] = -sin_psi
            R_GB[:,1,0] = sin_psi
            R_GB[:,2,2] = 1
        except:
            R_GB = np.array([[np.cos(psi), -np.sin(psi), 0],
                             [np.sin(psi),  np.cos(psi), 0],
                             [          0,            0, 1]])
        return R_GB
    
    @staticmethod
    def R_BG(psi):
        try:
            R_BG = np.zeros((len(psi),3,3))
            cos_psi = np.cos(psi)
            sin_psi = np.sin(psi)
            R_BG[:,0,0] = cos_psi
            R_BG[:,1,1] = cos_psi
            R_BG[:,0,1] = sin_psi
            R_BG[:,1,0] = -sin_psi
            R_BG[:,2,2] = 1
        except:
            R_BG = np.array([[ np.cos(psi),  np.sin(psi), 0],
                             [-np.sin(psi),  np.cos(psi), 0],
                             [           0,            0, 1]])
        return R_BG  

def sample_init_target(sample_time=False, obs=ff.obs_nominal):

    # Sample initial and final states
    valid = False
    while not valid:
        state_init = np.random.uniform(low=[ff.start_region['xy_low'][0], ff.start_region['xy_low'][1], -np.pi, 0, 0, 0],
                                   high=[ff.start_region['xy_up'][0], ff.start_region['xy_up'][1], np.pi, 0, 0, 0])
        valid = ((np.linalg.norm(state_init[:2] - obs['position'], axis=1) - (obs['radius'] + ff.robot_radius)*ff.safety_margin) > 0).all()
    valid = False
    while not valid:
        state_target = np.random.uniform(low=[ff.goal_region['xy_low'][0], ff.goal_region['xy_low'][1], -np.pi, 0, 0, 0],
                                     high=[ff.goal_region['xy_up'][0], ff.goal_region['xy_up'][1], np.pi, 0, 0, 0])
        valid = ((np.linalg.norm(state_target[:2] - obs['position'], axis=1) - (obs['radius'] + ff.robot_radius)*ff.safety_margin) > 0).all() & ((np.linalg.norm(state_target[:2] - state_init[:2]) - ff.min_init_dist) > 0)
    # Select correct value of final angle
    psi_period = np.array([0, -2*np.pi, 2*np.pi])
    psi_dist = np.abs(state_init[2] - (state_target[2] + psi_period))
    state_target[2] = state_target[2] + psi_period[np.argmin(psi_dist)]
    
    # Eventually sample final time
    if sample_time:
        final_time = np.random.choice(ff.final_time_choices)
        return state_init, state_target, final_time
    else:
        return state_init, state_target

def generate_random_obstacles(num_obstacles=2, xy_low=ff.obs_region['xy_low'], xy_up=ff.obs_region['xy_up'],
                              radii_range=(0.1, 0.2), uniform_radius=False, fixed_radius=None, prevent_overlap=True,
                              max_iter=1000, margin_between_obstacles=0., margin_to_boundaries=0.):
    """
    Generate a set of random obstacles within specified 2D table limits, ensuring that the entire obstacle (including
    its radius and specified margins) fits within the table boundaries and optionally preventing overlapping
    obstacles. If obstacles cannot be placed without overlap after a number of iterations, an error is raised.

    Args:
        num_obstacles (int): Number of obstacles to generate.
        xy_low (np.array): Lower bounds (x, y) of the table area.
        xy_up (np.array): Upper bounds (x, y) of the table area.
        radii_range (tuple): Min and max radii of obstacles.
        uniform_radius (bool): If True, all obstacles have the same radius.
        fixed_radius (float): Fixed radius for all obstacles if uniform_radius is True.
        prevent_overlap (bool): If True, ensures that obstacles do not overlap.
        max_iter (int): Maximum number of attempts to place each obstacle without overlapping.
        margin_between_obstacles (float): Minimum required margin between any two obstacles.
        margin_to_boundaries (float): Minimum required margin between obstacles and table boundaries.

    Returns:
        dict: A dictionary containing 'position' and 'radius' keys with numpy arrays.
    """
    radii = np.full((num_obstacles,), fixed_radius) if uniform_radius and fixed_radius is not None \
        else np.random.uniform(low=radii_range[0], high=radii_range[1], size=(num_obstacles,))
    positions = np.zeros((num_obstacles, 2))

    for i in range(num_obstacles):
        effective_radius = radii[i] + margin_to_boundaries
        adjusted_xy_low = xy_low + effective_radius
        adjusted_xy_up = xy_up - effective_radius
        for _ in range(max_iter):
            candidate_position = np.random.uniform(low=adjusted_xy_low, high=adjusted_xy_up)

            # Check for overlap if required, considering the margin between obstacles
            if prevent_overlap and i > 0:
                min_distance = (radii[i] + radii[:i] + 2 * ff.robot_radius) * ff.safety_margin + margin_between_obstacles
                if any(np.linalg.norm(candidate_position - positions[:i], axis=1) < min_distance):
                    continue  # If overlap or insufficient margin occurs, try again
            positions[i] = candidate_position
            break
        else:
            # If no valid position is found after max_iter attempts
            raise ValueError(
                f"Unable to place obstacles without overlap after {max_iter} attempts. Consider reducing the number "
                f"or size of obstacles, increasing the table area, or adjusting the margins.")

    return {'position': positions, 'radius': radii}

def generate_perfect_observations(positions, radii):
    """
    Generate a flat array containing position coordinates followed by radius for each observation.

    Args:
    positions (numpy.ndarray): An array of shape (n, 2) where n is the number of observations, and each element is an (x, y) tuple.
    radii (numpy.ndarray): An array of shape (n,) containing radii corresponding to each position.

    Returns:
    numpy.ndarray: A flat array of shape (3n,) where each group of three elements contains the x coordinate, y coordinate, and radius for each observation.
    """
    n_obs = len(radii)
    out = np.empty(shape=(ff.SINGLE_OBS_DIM * n_obs), dtype=float)
    out[0::ff.SINGLE_OBS_DIM] = positions[:, 0]  # x coordinates
    out[1::ff.SINGLE_OBS_DIM] = positions[:, 1]  # y coordinates
    out[2::ff.SINGLE_OBS_DIM] = radii  # radii

    return out

def compute_relative_observations(ff_pos, obs_pos, obs_radius_enlarged):
    '''
        Generate a flat array containing relative position of obstacles and distance to their borders (enlarged by robot_radius and safety_margin).
        ABSOLUTE OBS:                             RELATIVE OBS:
        1) x_obs                -->               1) x_obs - x_ff
        2) y_obs                -->               2) y_obs - y_ff
        3) R_obs                -->               3) ((x_obs - x_ff)^2 + (y_obs - y_ff)^2)^(1/2) - radius_enlarged ( radius_enlarged = (R_obs + R_robot)*safety_margin)
    '''
    N_obs = obs_radius_enlarged.shape[0]
    rel_observation = np.zeros((N_obs * ff.SINGLE_OBS_DIM,))
    for n_obs in range(N_obs):
        i_pos = [n_obs*ff.SINGLE_OBS_DIM, n_obs*ff.SINGLE_OBS_DIM + 1]
        i_R = n_obs*ff.SINGLE_OBS_DIM + 2
        rel_observation[i_pos] = obs_pos[n_obs, :] - ff_pos
        rel_observation[i_R] = (rel_observation[i_pos[0]]**2 + rel_observation[i_pos[1]]**2)**(1/2) - obs_radius_enlarged[n_obs]
    
    return rel_observation

# ----------------------------------
# Optimization problems
def ocp_no_obstacle_avoidance(model:FreeflyerModel, state_init, state_final, final_time=ff.T_nominal, obs=ff.obs_nominal):
    # Initial reference
    state_ref, action_ref = model.initial_guess_line(state_init, state_final, final_time)
    # Validate the format and structure of obstacle data before processing.
    obs = copy.deepcopy(obs)
    check_obs_format(obs)
    obs['radius'] = (obs['radius'] + model.param['radius'])*ff.safety_margin

    # Initial condition for the scp
    DELTA_J = 10
    trust_region = ff.trust_region0
    beta_SCP = (ff.trust_regionf/ff.trust_region0)**(1/ff.iter_max_SCP)
    J_vect = np.ones(shape=(ff.iter_max_SCP,), dtype=float)*1e12

    for scp_iter in range(ff.iter_max_SCP):
        # define and solve
        states_scp, actions_scp, J, feas_scp = model.ocp_scp(
            state_ref, action_ref, state_init, state_final, obs, trust_region, obs_av=False)
        if feas_scp == 'infeasible':
            break
        J_vect[scp_iter] = J

        # compute error
        trust_error = np.max(np.linalg.norm(states_scp - state_ref, axis=0))
        if scp_iter > 0:
            DELTA_J = J_prev - J

        # Update iterations
        state_ref = states_scp
        action_ref = actions_scp
        J_prev = J
        trust_region = beta_SCP*trust_region
        if scp_iter >= 1 and (trust_error <= ff.trust_regionf and abs(DELTA_J) < ff.J_tol):
            break

    if feas_scp == 'infeasible':
        s_opt = None
        a_opt = None
        a_opt_t = None
        J = None
    else:
        s_opt = states_scp
        a_opt = actions_scp
        a_opt_t = model.param['Lambda_inv'] @ (model.R_BG(s_opt[2,:-1]) @ a_opt[:,None,:].transpose(2,0,1))[:,:,0].T
        J = J_vect[scp_iter]

    traj_opt = {
        'time' : np.arange(0,final_time + ff.dt/2, ff.dt),
        'states' : s_opt,
        'actions_G' : a_opt,
        'actions_t' : a_opt_t
    }

    return traj_opt, J, scp_iter, feas_scp

def ocp_obstacle_avoidance(model:FreeflyerModel, state_ref, action_ref, state_init, state_final, obs=ff.obs_nominal):
    # Initalization
    # Validate the format and structure of obstacle data before processing.
    obs = copy.deepcopy(obs)
    check_obs_format(obs)
    obs['radius'] = (obs['radius'] + model.param['radius'])*ff.safety_margin
    final_time = ff.dt*(state_ref.shape[1] - 1)

    # Initial condition for the scp
    DELTA_J = 10
    trust_region = ff.trust_region0
    beta_SCP = (ff.trust_regionf/ff.trust_region0)**(1/ff.iter_max_SCP)
    J_vect = np.ones(shape=(ff.iter_max_SCP,), dtype=float)*1e12

    for scp_iter in range(ff.iter_max_SCP):
        # define and solve
        states_scp, actions_scp, J, feas_scp = model.ocp_scp(
            state_ref, action_ref, state_init, state_final, obs, trust_region, obs_av=True)
        if feas_scp == 'infeasible':
            break
        J_vect[scp_iter] = J

        # compute error
        trust_error = np.max(np.linalg.norm(states_scp - state_ref, axis=0))
        if scp_iter > 0:
            DELTA_J = J_prev - J

        # Update iterations
        state_ref = states_scp
        action_ref = actions_scp
        J_prev = J
        trust_region = beta_SCP*trust_region
        if scp_iter >= 1 and (trust_error <= ff.trust_regionf and abs(DELTA_J) < ff.J_tol):
            break

    if feas_scp == 'infeasible':
        s_opt = None
        a_opt = None
        a_opt_t = None
        J = None
    else:
        s_opt = states_scp
        a_opt = actions_scp
        a_opt_t = model.param['Lambda_inv'] @ (model.R_BG(s_opt[2,:-1]) @ a_opt[:,None,:].transpose(2,0,1))[:,:,0].T
        J = J_vect[scp_iter]

    traj_opt = {
        'time' : np.arange(0,final_time + ff.dt/2, ff.dt),
        'states' : s_opt,
        'actions_G' : a_opt,
        'actions_t' : a_opt_t
    }

    return traj_opt, J_vect, scp_iter, feas_scp

# Reward to go and constraints to go
def compute_reward_to_go(actions):
    if len(actions.shape) == 2:
        actions = actions[None,:,:]
    n_data, n_time = actions.shape[0], actions.shape[1]
    rewards_to_go = np.empty(shape=(n_data, n_time), dtype=float)
    for n in range(n_data):
        for t in range(n_time):
            rewards_to_go[n, t] = - np.sum(np.linalg.norm(actions[n, t:, :], ord=1,  axis=1))

    return rewards_to_go

def compute_constraint_to_go(states, obs_positions, obs_radii, n_obs=ff.n_obs_nominal):
    if len(states.shape) == 2:
        states = states[None,:,:]
    if len(obs_positions.shape) == 2:
        obs_positions = obs_positions[None,:,:]
        obs_radii = obs_radii[None,:]
        n_obs = np.array([n_obs])
    n_data, n_time = states.shape[0], states.shape[1]
    constraint_to_go = np.empty(shape=(n_data, n_time), dtype=float)
    for n in range(n_data):
        constr_koz_n, constr_koz_violation_n = check_koz_constraint(states[n,:,:], obs_positions[n,:n_obs[n],:], obs_radii[n,:n_obs[n]])
        constraint_to_go[n,:] = np.array([np.sum(constr_koz_violation_n[:,t:]) for t in range(n_time)])

    return constraint_to_go

def check_koz_constraint(states, obs_positions, obs_radii):
    constr_koz = np.linalg.norm(states[None,:,:2] - obs_positions[:,None,:], axis=2) - obs_radii[:,None]
    constr_koz_violation = 1*(constr_koz < -1e-6)

    return constr_koz, constr_koz_violation

def check_obs_format(obs):
    """
    Validate the format of the 'obs' dictionary to ensure it correctly defines obstacles.

    This function checks that 'obs' is a dictionary with the necessary keys 'position' and 'radius',
    where 'position' is a 2D numpy array of obstacle coordinates, and 'radius' is a 1D array of
    their corresponding radii. Each element in 'position' must have a corresponding radius in 'radius'.

    Raises:
        ValueError: If obs is not a dictionary or the array dimensions do not meet requirements.
        KeyError: If the necessary keys are missing from the dictionary.
        TypeError: If the elements are not numpy arrays.
    """

    if not isinstance(obs, dict):
        raise ValueError("obs must be a dictionary.")

    if 'position' not in obs or 'radius' not in obs:
        raise KeyError("obs must contain 'position' and 'radius' keys.")

    if not (isinstance(obs['position'], np.ndarray) and isinstance(obs['radius'], np.ndarray)):
        raise TypeError("Both 'position' and 'radius' must be numpy arrays.")

    if obs['position'].ndim != 2 or obs['position'].shape[1] != 2:
        raise ValueError("obs['position'] must be a 2D numpy array with two columns (x, y coordinates).")

    if obs['radius'].ndim != 1:
        raise ValueError("obs['radius'] must be a 1D numpy array.")

    if obs['position'].shape[0] != obs['radius'].size:
        raise ValueError("The number of positions must match the number of radii.")
