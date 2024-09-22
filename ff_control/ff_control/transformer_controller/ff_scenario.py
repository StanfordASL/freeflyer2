import os
import sys

root_folder = os.path.abspath(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.append(root_folder)

import numpy as np

# Problem dimensions
N_STATE = 6
N_ACTION = 3
N_CLUSTERS = 4
N_OBS_MAX = 4
SINGLE_OBS_DIM = 3
N_OBSERVATION = N_OBS_MAX*SINGLE_OBS_DIM
TRANSFORMER_MODEL = 'checkpoint_ff_time40_100_chunk100R_ctgrtg'#'checkpoint_ff_obs_4scen_rel_ctgrtg'#'checkpoint_ff_time40_100_chunk100R_ctgrtg'#'checkpoint_ff_ctgrtg'#

# Generalization level
generalized_time = True
generalized_obs = False

# time problem constants
if generalized_time:
    chunksize = 100 #None/100
    random_chunk = True
else:
    S = 101 # number of control switches
    n_time_rpod = S-1
    chunksize = None
    random_chunk = None

# constants
mass = 16.0
inertia = 0.18
robot_radius = 0.2
F_max_per_thruster = 0.2
thrusters_lever_arm = 0.11461
Lambda = np.array([[                  0,                    1,                    0,                   1],
                   [                  1,                    0,                    1,                   0],
                   [thrusters_lever_arm, -thrusters_lever_arm, -thrusters_lever_arm, thrusters_lever_arm]])
Lambda_inv = np.array([[  0, 0.5,  1/(4*thrusters_lever_arm)],
                       [0.5,   0, -1/(4*thrusters_lever_arm)],
                       [  0, 0.5, -1/(4*thrusters_lever_arm)],
                       [0.5,   0,  1/(4*thrusters_lever_arm)]])

# Table, start and goal regions dimensions
table = {
    'xy_low' : np.array([0.,0.]),
    'xy_up' : np.array([3.5, 2.5])
}
start_region = {
    'xy_low' : table['xy_low'] + robot_radius,
    'xy_up' : np.array([0.5, 2.5]) - robot_radius
}
goal_region = {
    'xy_low' : np.array([3.0, 0.]) + robot_radius,
    'xy_up' : table['xy_up'] - robot_radius
}
obs_region = {
    'xy_low': np.array([0.5, 0.]),
    'xy_up': np.array([3.0, 2.5])
}
min_init_dist = 0.5

# Time discretization and bounds
dt = 0.5 if generalized_time else 0.4
# T_const = 40.0 # max final time horizon in sec
'''T_min = T_const if dataset_scenario == 'time_constant' else (10.0 if dataset_scenario == 'time_whole_table' else 20.0)
T_max = T_const if dataset_scenario == 'time_constant' else 100.0
final_time_choices = np.arange(T_min, T_max+dt/2, dt)'''
T_min = 40.0 if generalized_time else 40.0
T_max = 100.0 if generalized_time else 40.0
T_nominal = 80.0
final_time_choices = np.arange(T_min, T_max+1, 20.0)
n_time_max = int(T_max/dt)

# Obstacle
obs_nominal = {
    'position' : np.array([[1.0,  0.7],
                           [1.5,  1.7],
                           [2.5, 0.75],
                           [2.5, 1.75]]),
    'radius' : np.array([0.2, 0.2, 0.2, 0.2])
}
n_obs_nominal = obs_nominal['position'].shape[0]
relative_observations = True #True/False
if generalized_obs:
    obs1 = {
        'position': np.array([[1.0, 0.7],
                              [1.5, 1.7],
                              [2.5, 0.75],
                              [2.5, 1.75]]),
        'radius': np.array([0.18, 0.15, 0.12, 0.2])
    }
    obs2 = {
        'position': np.array([[.8, .5],
                              [1.3, 1.7],
                              [2, 1.05],
                              [2.6, 2.05]]),
        'radius': np.array([0.17, 0.2, 0.14, 0.13])
    }
    obs3 = {
        'position': np.array([[1.5, .6],
                              [2.1, 1.13],
                              [1.1, 1.45],
                              [1.9, 2.05]]),
        'radius': np.array([0.138, 0.11, 0.15, 0.19])
    }
    obs4 = {
        'position': np.array([[1, 1],
                              [1.75, 1.5],
                              [2.5, 2],
                              [2.4, .45]]),
        'radius': np.array([0.123, 0.151, 0.131, 0.184])
    }
    obs_list = [obs1, obs2, obs3, obs4]
    n_obs_list = [4, 4, 4, 4]
    obs_nominal = obs1 # Set this variable to the desired obs configuations (one of the four above)
safety_margin = 1.1

'''# PID Controller
gain_f = 2.0*0
gain_df = 10.0*2
gain_t = 0.2*0
gain_dt = 0.4*2
K = np.array([[gain_f, 0, 0, gain_df, 0, 0],
              [0, gain_f, 0, 0, gain_df, 0],
              [0, 0, gain_t, 0, 0, gain_dt]])'''

# Optimization interface
iter_max_SCP = 20
trust_region0 = 10.
trust_regionf = 0.005
J_tol = 10**(-6)