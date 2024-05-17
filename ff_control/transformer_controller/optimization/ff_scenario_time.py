import os
import sys

root_folder = os.path.abspath(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.append(root_folder)

import numpy as np

# Problem dimensions
N_STATE = 6
N_ACTION = 3
N_CLUSTERS = 4

# time problem constants
'''S = 101 # number of control switches
n_time_rpod = S-1'''
chunksize = 100 #None

# constants
mass = 16.0
inertia = 0.18
robot_radius = 0.15
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
dataset_scenario = 'time' #'time_whole_table' #
table = {
    'xy_low' : np.array([0.,0.]),
    'xy_up' : np.array([3.5, 2.5])
}
if dataset_scenario == 'time_whole_table':
    start_region = {
        'xy_low' : table['xy_low'] + robot_radius,
        'xy_up' : np.array([3.5, 2.5]) - robot_radius
    }
    goal_region = {
        'xy_low' : np.array([0., 0.]) + robot_radius,
        'xy_up' : table['xy_up'] - robot_radius
    }
elif dataset_scenario == 'time':
    start_region = {
        'xy_low' : table['xy_low'] + robot_radius,
        'xy_up' : np.array([0.5, 2.5]) - robot_radius
    }
    goal_region = {
        'xy_low' : np.array([3.0, 0.]) + robot_radius,
        'xy_up' : table['xy_up'] - robot_radius
    }
else:
    raise NameError('dataset_scenario not recognized!')
min_init_dist = 0.5

# Time discretization and bounds
dt = 0.5
T_min = 10.0 if dataset_scenario == 'time_whole_table' else 20.0
T_max = 100.0
final_time_choices = np.arange(T_min, T_max+dt/2, dt)
'''T = 40.0 # max final time horizon in sec'''
n_time_max = int(T_max/dt)

# Obstacle
obs = {
    'position' : np.array([[1.0,  0.7],
                           [1.5,  1.7],
                           [2.5, 0.75],
                           [2.5, 1.75]]),
    'radius' : np.array([0.2, 0.2, 0.2, 0.2])
}
n_obs = obs['position'].shape[0]
safety_margin = 1.1

# PID Controller
gain_f = 2.0*0
gain_df = 10.0*2
gain_t = 0.2*0
gain_dt = 0.4*2
K = np.array([[gain_f, 0, 0, gain_df, 0, 0],
              [0, gain_f, 0, 0, gain_df, 0],
              [0, 0, gain_t, 0, 0, gain_dt]])

# Optimization interface
iter_max_SCP = 20
trust_region0 = 10.
trust_regionf = 0.005
J_tol = 10**(-6)