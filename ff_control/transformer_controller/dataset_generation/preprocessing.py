import os
import sys
import argparse

root_folder = os.path.abspath(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.append(root_folder)

import numpy as np
import copy
import torch

from dynamics.freeflyer import compute_constraint_to_go, compute_reward_to_go
from optimization.ff_scenario import robot_radius, safety_margin, N_STATE, N_ACTION, SINGLE_OBS_DIM, n_time_max, obs_nominal, \
    dataset_scenario, relative_observations, generalized_time, generalized_obs

def compute_relative_observations(abs_observations, abs_states):
    '''
        Fuction to compute relative observations with respect to the obstacles, if required when generating the dataset.
        ABSOLUTE OBS:                             RELATIVE OBS:
        1) x_obs                -->               1) x_obs - x_ff
        2) y_obs                -->               2) y_obs - y_ff
        3) R_obs                -->               3) ((x_obs - x_ff)^2 + (y_obs - y_ff)^2)^(1/2) - (R_obs + R_robot)*safety_margin
    '''
    rel_observations = np.zeros_like(abs_observations)
    for n in range(abs_observations.shape[0]):
        for n_obs in range(data_param['n_obs'][n]):
            i_pos = [n_obs*SINGLE_OBS_DIM, n_obs*SINGLE_OBS_DIM + 1]
            i_R = n_obs*SINGLE_OBS_DIM + 2
            rel_observations[n,:,i_pos] = (abs_observations[n,:,i_pos].T - abs_states[n,:,:2]).T
            rel_observations[n,:,i_R] = (rel_observations[n,:,i_pos[0]]**2 + rel_observations[n,:,i_pos[1]]**2)**(1/2) - (abs_observations[n,:,i_R] + robot_radius)*safety_margin
        if n%1000 == 0:
            print(n)
    return rel_observations

if generalized_time or generalized_obs:
    dataset_scenario_folder = '/' + dataset_scenario
else:
    dataset_scenario = ''
parser = argparse.ArgumentParser(description='transformer-ff')
parser.add_argument('--data_dir', type=str, default='dataset' + dataset_scenario_folder,
                    help='defines directory from where to load files')
parser.add_argument('--data_dir_torch', type=str, default='dataset' + dataset_scenario_folder + '/torch/v05',  # remember to change v0# depending on the dataset version loaded
                    help='defines directory from where to load files')
args = parser.parse_args()
args.data_dir = root_folder + '/' + args.data_dir
args.data_dir_torch = root_folder + '/' + args.data_dir_torch

print('Loading data...', end='')

data_scp = np.load(args.data_dir + '/dataset-ff-v05-scp.npz', allow_pickle=True)
data_cvx = np.load(args.data_dir + '/dataset-ff-v05-cvx.npz', allow_pickle=True)
data_param = np.load(args.data_dir + '/dataset-ff-v05-param.npz', allow_pickle=True) 

# Import datatsets from all file
if generalized_time:
    states_scp_raw = data_scp['states_scp']
    actions_scp_raw = data_scp['actions_scp']
    states_cvx_raw = data_cvx['states_cvx']
    actions_cvx_raw = data_cvx['actions_cvx']
    target_state = data_param['target_state']
    final_time = data_param['final_time']
    print('States shapes', states_scp_raw.shape[0])

    # Create extended sequences with equal length in the time axis
    states_scp = -100*np.ones((states_scp_raw.shape[0], n_time_max, N_STATE))
    actions_scp = -100*np.ones((actions_scp_raw.shape[0], n_time_max, N_ACTION))
    states_cvx = -100*np.ones((states_cvx_raw.shape[0], n_time_max, N_STATE))
    actions_cvx = -100*np.ones((actions_cvx_raw.shape[0], n_time_max, N_ACTION))

    # Fill them extending the states and actions sequences with target values
    for n_data, (states_scp_i, actions_scp_i, states_cvx_i, actions_cvx_i) in enumerate(zip(states_scp_raw, actions_scp_raw, states_cvx_raw, actions_cvx_raw)):
        states_scp[n_data,:,:] = np.vstack((states_scp_i, np.repeat(target_state[n_data:n_data+1,:], n_time_max - states_scp_i.shape[0], axis=0)))
        states_cvx[n_data,:,:] = np.vstack((states_cvx_i, np.repeat(target_state[n_data:n_data+1,:], n_time_max - states_cvx_i.shape[0], axis=0)))

        actions_scp[n_data,:,:] = np.vstack((actions_scp_i, np.repeat(np.zeros((1,N_ACTION)), n_time_max - actions_scp_i.shape[0], axis=0)))
        actions_cvx[n_data,:,:] = np.vstack((actions_cvx_i, np.repeat(np.zeros((1,N_ACTION)), n_time_max - actions_cvx_i.shape[0], axis=0)))

else:
    states_scp = data_scp['states_scp']
    actions_scp = data_scp['actions_scp']
    states_cvx = data_cvx['states_cvx']
    actions_cvx = data_cvx['actions_cvx']

# Save the states and actions torch tensors
torch_states_scp = torch.from_numpy(states_scp)
torch.save(torch_states_scp, args.data_dir_torch + '/torch_states_scp.pth')
torch_states_cvx = torch.from_numpy(states_cvx)
torch.save(torch_states_cvx, args.data_dir_torch + '/torch_states_cvx.pth')

torch_actions_scp = torch.from_numpy(actions_scp)
torch.save(torch_actions_scp, args.data_dir_torch + '/torch_actions_scp.pth')
torch_actions_cvx = torch.from_numpy(actions_cvx)
torch.save(torch_actions_cvx, args.data_dir_torch + '/torch_actions_cvx.pth')

# Pre-compute torch observations
if generalized_obs:
    observations_scp = data_scp['observations_scp']
    torch_observations_scp = torch.from_numpy(observations_scp)
    torch.save(torch_observations_scp, args.data_dir_torch + '/torch_observations_scp.pth')

    observations_cvx = data_cvx['observations_cvx']
    torch_observations_cvx = torch.from_numpy(observations_cvx)
    torch.save(torch_observations_cvx, args.data_dir_torch + '/torch_observations_cvx.pth')

    if relative_observations:
        rel_observations_scp = compute_relative_observations(observations_scp, states_scp)
        torch_rel_observations_scp = torch.from_numpy(rel_observations_scp)
        torch.save(torch_rel_observations_scp, args.data_dir_torch + '/torch_rel_observations_scp.pth')

        rel_observations_cvx = compute_relative_observations(observations_cvx, states_cvx)
        torch_rel_observations_cvx = torch.from_numpy(rel_observations_cvx)
        torch.save(torch_rel_observations_cvx, args.data_dir_torch + '/torch_rel_observations_cvx.pth')

# Pre-compute torch rewards to go and constraints to go
torch_rtgs_scp = torch.from_numpy(compute_reward_to_go(actions_scp))
torch.save(torch_rtgs_scp, args.data_dir_torch + '/torch_rtgs_scp.pth')

torch_rtgs_cvx = torch.from_numpy(compute_reward_to_go(actions_cvx))
torch.save(torch_rtgs_cvx, args.data_dir_torch + '/torch_rtgs_cvx.pth')

if generalized_obs:
    n_obs = data_param['n_obs']
    obs_position = data_param['obs_position']
    obs_radius = (data_param['obs_radius'] + robot_radius)*safety_margin
else:
    obs = copy.deepcopy(obs_nominal)
    n_obs = len(obs['radius'])
    obs_position = obs['position']
    obs_radius = (obs['radius'] + robot_radius)*safety_margin

torch_ctgs_scp = torch.from_numpy(compute_constraint_to_go(states_scp, obs_position, obs_radius, n_obs))
torch.save(torch_ctgs_scp, args.data_dir_torch + '/torch_ctgs_scp.pth')

torch_ctgs_cvx = torch.from_numpy(compute_constraint_to_go(states_cvx, obs_position, obs_radius, n_obs))
torch.save(torch_ctgs_cvx, args.data_dir_torch + '/torch_ctgs_cvx.pth')

# Permutation
if states_cvx.shape[0] != states_scp.shape[0]:
    raise RuntimeError('Different dimensions of cvx and scp datasets.')
perm = np.random.permutation(states_cvx.shape[0]*2)
np.save(args.data_dir_torch + '/permutation.npy', perm)

print('Completed\n')

# IDEA to speed up things if needed

# def do_preprocessing(states_rtn, actions, oe, dt, n_data, n_time):

#     constraint_to_go = np.empty(shape=(n_data, n_time), dtype=float)
#     rewards_to_go = np.empty(shape=(n_data, n_time), dtype=float)
#     stm_roe  = np.empty((n_data, n_time, 6, 6))
#     cim_roe  = np.empty((n_data, n_time, 6, 3))
#     stm_rtn  = np.empty((n_data, n_time, 6, 6))
#     cim_rtn  = np.empty((n_data, n_time, 6, 3))

#     for n in range(n_data):

#         constr_koz_n, constr_koz_violation_n = check_koz_constraint(np.transpose(np.squeeze(states_rtn[n, :, :])), n_time)
#         r_tot_n = np.sum(la.norm(actions[n, :, :], axis=1))

#         for t in range(n_time):

#             constraint_to_go[n, t] = np.sum(constr_koz_violation_n[t:])
#             rewards_to_go[n, t] = -np.sum(la.norm(actions[n, t:, :], axis=1)) / r_tot_n
#             stm_roe[n,t] = state_transition(oe[n, t], dt[n])
#             cim_roe[n,t] = control_input_matrix(oe[n, t])
#             map_t = map_mtx_roe_to_rtn(oe[n, t])
#             if t < n_time - 1:
#                 map_t_new = map_mtx_roe_to_rtn(oe[n, t+1])
#             else: 
#                 a = oe[n, t][0]
#                 nn = np.sqrt(mu_E/a**3)
#                 oe_new = oe[n, t] + np.array([0, 0, 0, 0, 0, nn*dt.item(n)]).reshape((6,))
#                 map_t_new = map_mtx_roe_to_rtn(oe_new)
#             stm_rtn[n,t] = np.matmul(map_t_new, np.matmul(stm_roe[n,t], np.linalg.inv(map_t)))
#             cim_rtn[n,t] = np.matmul(map_t, cim_roe[n,t])

#     return constraint_to_go
