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

# Generalization level
generalized_time = False
generalized_obs = True
embed_entire_observation = False

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
dataset_scenario = 'var_obstacles_rand_scenarios' #'time_whole_table' #
table = {
    'xy_low' : np.array([0., 0.]),
    'xy_up' : np.array([3.5, 2.5])
}
if 'whole_table' in dataset_scenario:
    start_region = {
        'xy_low' : table['xy_low'] + robot_radius,
        'xy_up' : np.array([3.5, 2.5]) - robot_radius
    }
    goal_region = {
        'xy_low' : np.array([0., 0.]) + robot_radius,
        'xy_up' : table['xy_up'] - robot_radius
    }
else:
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
'''T_min = T_const if dataset_scenario == 'time_constant' else (10.0 if dataset_scenario == 'time_whole_table' else 20.0)
T_max = T_const if dataset_scenario == 'time_constant' else 100.0
final_time_choices = np.arange(T_min, T_max+dt/2, dt)'''
T_min = 40.0 if generalized_time else 40.0
T_max = 100.0 if generalized_time else 40.0
T_nominal = 40.0 # nominal final time horizon in sec
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
    '''obs1 = {
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
    n_obs_list = [4, 4, 4, 4]'''
    obs0 = {
        'position' : np.array([[2.5,  1.3],
                            [1.2,  1.2],
                            [2.0,  0.7],
                            [1.9, 1.75]]),
        'radius' : np.array([0.13, 0.2, 0.16, 0.2])
    }
    obs1 = {
        'position' : np.array([[2.5,  1.9],
                            [1.7, 1.75],
                            [1.2,  0.9],
                            [2.2,  0.7]]),
        'radius' : np.array([0.15, 0.2, 0.18, 0.12])
    }
    obs2 = {
        'position' : np.array([[1.6,  1.4],
                            [0.9, 1.75],
                            [2.5,  1.9],
                            [2.2,  0.7]]),
        'radius' : np.array([0.17, 0.19, 0.14, 0.2])
    }
    obs3 = {
        'position' : np.array([[1.3,  0.7],
                            [2.6, 1.75],
                            [1,  1.9],
                            [1.9,  1.4]]),
        'radius' : np.array([0.2, 0.19, 0.14, 0.17])
    }
    obs4 = {
        'position' : np.array([[1.3,  1.0],
                            [2.8,  1.3],
                            [2.2,  0.7],
                            [1.6, 1.75]]),
        'radius' : np.array([0.16, 0.13, 0.2, 0.23])
    }
    obs5 = {
        'position' : np.array([[.8, .5],
                            [1.3, 1.7],
                            [2, 1.05],
                            [2.6, 2.05]]),
        'radius' : np.array([0.17, 0.2, 0.14, 0.13])
    }
    obs6 = {
        'position' : np.array([[1.5, .6],
                            [2.1, 1.13],
                            [1.1, 1.45],
                            [1.9, 2.05]]),
        'radius' : np.array([0.138, 0.11, 0.15, 0.19])
    }
    obs7 = {
        'position' : np.array([[1, 1],
                            [1.75, 1.5],
                            [2.5, 2],
                            [2.4, .45]]),
        'radius' : np.array([0.123, 0.151, 0.131, 0.184])
    }
    obs8 = {
        'position' : np.array([[1.6727547666191276, 1.2843406322905162],
                            [1.0840661300637007, 0.5361579292884463],
                            [1.3801072724175363, 1.9784890280459697],
                            [2.727819438528107, 1.1777855304127303]]),
        'radius' : np.array([ 0.15339303661300935, 0.16598369582592476, 0.10075645289194371, 0.15256289935419431])
    }
    obs9 = {
        'position' : np.array([[1.62689314, 1.98860954],
                            [0.88884833, 0.75615519],
                            [2.23800377, 0.45431808],
                            [2.06970026, 1.1674797 ]]),
        'radius' : np.array([0.18816975, 0.12672767, 0.11910985, 0.13555548])
    }
    obs10 = {
        'position' : np.array([[1.15370543, 1.09110425],
                            [1.34359307, 0.28624518],
                            [2.19302603, 1.79911688],
                            [2.48829184, 0.9253271 ]]),
        'radius' : np.array([0.15110599, 0.10785146, 0.19881381, 0.12667007])
    }
    obs11 = {
        'position' : np.array([[1.97157936, 1.23268527],
                            [1.24549497, 1.85344324],
                            [2.44949628, 0.49396605],
                            [2.63707386, 2.05951662]]),
        'radius' : np.array([0.15943664, 0.11171497, 0.10946028, 0.1661533 ])
    }
    obs12 = {
        'position' : np.array([[2.1873858648612146, 1.4195382901329396],
                            [1.513435454186436, 0.5164013896529343],
                            [1.1072122389517203, 2.0404009719543145],
                            [1.14933076823709, 1.1482519970784735]]),
        'radius' : np.array([ 0.18372475290849227, 0.11456488453657399, 0.1529624400595343, 0.12791000716427708])
    }
    obs13 = {
        'position' : np.array([[1.322254215490553, 1.8963692433323505],
                            [2.40300029441051, 0.37469491290811524],
                            [1.3152717303515336, 1.0562467429718106],
                            [2.6020031227216416, 1.2843504140538304]]),
        'radius' : np.array([ 0.15614304519088035, 0.15518705647808573, 0.17193008422083, 0.11564739755372316])
    }
    obs14 = {
        'position' : np.array([[1.8515708055695828, 0.6612535915615683],
                            [1.273302738904587, 1.0542174587757076],
                            [2.5707656352427, 1.285562098877053],
                            [2.192820108276734, 2.027946857026107]]),
        'radius' : np.array([ 0.1439455760282616, 0.15681929106417777, 0.1860621880796965, 0.17652270525313737])
    }
    obs15 = {
        'position' : np.array([[1.0, 0.7],
                            [1.5, 1.7],
                            [2.5, 0.75],
                            [2.5, 1.75]]),
        'radius' : np.array([0.18, 0.15, 0.12, 0.2])
    }
    obs16 = {
        'position' : np.array([[1.3, 1.1],
                            [2.2, 1.4],
                            [1, 1.9],
                            [2.4, .6]]),
        'radius' : np.array([0.18, 0.15, 0.12, 0.15])
    }
    obs17 = {
        'position' : np.array([[1, .7],
                            [1.8, 1.1],
                            [1, 1.8],
                            [2.5, 1.7]]),
        'radius' : np.array([0.13, 0.16, 0.14, 0.15])
    }
    obs18 = {
        'position' : np.array([[1, .7],
                            [2.5, .8],
                            [1.6, 1.35],
                            [2.5, 1.7]]),
        'radius' : np.array([0.13, 0.16, 0.14, 0.15])
    }
    obs_list = [obs0, obs1, obs2, obs3, obs4, obs5, obs6, obs7, obs8, obs9, obs10, obs11, obs12, obs13, obs14, obs15, obs16, obs17, obs18]
    n_obs_list = [4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4]
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