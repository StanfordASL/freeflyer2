import os
import sys
import copy

root_folder = os.path.abspath(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.append(root_folder)

from dynamics.freeflyer import FreeflyerModel, sample_init_target, ocp_no_obstacle_avoidance, \
    ocp_obstacle_avoidance, generate_random_obstacles, generate_perfect_observations
from optimization.ff_scenario import N_STATE, N_ACTION, N_CLUSTERS, N_OBS_MAX, T_nominal, dt, \
    dataset_scenario, generalized_obs, generalized_time, obs_list, obs_nominal
import numpy as np
from multiprocessing import Pool, set_start_method
import itertools
from tqdm import tqdm

def for_computation(input):
    # Input unpacking
    current_data_index = input[0]
    other_args = input[1]
    ff_model = other_args['ff_model']
    scenario_rand = other_args['scenario_rand']

    # Randomic sample of initial and final conditions
    if generalized_time and (not generalized_obs):
        init_state, target_state, final_time = sample_init_target(sample_time=True)
        obs = obs_nominal
    elif generalized_obs and (not generalized_time):
        init_state, target_state = sample_init_target()
        final_time = T_nominal
        if scenario_rand == 'random':
            num_obstacles = 4
            obs = generate_random_obstacles(num_obstacles=num_obstacles)
        elif scenario_rand == 'list':
            obs_index = np.random.choice(np.arange(0, 4))
            obs = copy.deepcopy(obs_list[obs_index])
    elif (not generalized_time) and (not generalized_obs):
        init_state, target_state = sample_init_target()
        obs = obs_nominal
        final_time = T_nominal
    else:
        raise ValueError('Generalized obstacles and final time not implemented yet!!')

    # Output dictionary initialization
    out = {'feasible' : True,
           'states_cvx' : [],
           'actions_cvx' : [],
           'actions_t_cvx' : [],
           'states_scp': [],
           'actions_scp' : [],
           'actions_t_scp' : [],
           'target_state' : [],
           'dtime' : [],
           'final_time' : [],
           'time' : [],
           'obstacles' : []
           }

    # Solve simplified problem -> without obstacle avoidance
    traj_cvx_i, J_cvx_i, iter_cvx_i, feas_cvx_i = ocp_no_obstacle_avoidance(ff_model, init_state, target_state, final_time, obs)

    if np.char.equal(feas_cvx_i,'optimal'):

        #  Solve scp with obstacles
        try:
            traj_scp_i, J_scp_i, iter_scp_i, feas_scp_i, = ocp_obstacle_avoidance(ff_model, traj_cvx_i['states'],
                                                                                  traj_cvx_i['actions_G'],
                                                                                  init_state,
                                                                                  target_state,
                                                                                  obs)

            if np.char.equal(feas_scp_i,'optimal'):
                # Save cvx and scp problems in the output dictionary
                out['states_cvx'] = np.transpose(traj_cvx_i['states'][:, :-1])
                out['actions_cvx'] = np.transpose(traj_cvx_i['actions_G'])
                out['actions_t_cvx'] = np.transpose(traj_cvx_i['actions_t'])
                
                out['states_scp'] = np.transpose(traj_scp_i['states'][:, :-1])
                out['actions_scp'] = np.transpose(traj_scp_i['actions_G'])
                out['actions_t_scp'] = np.transpose(traj_scp_i['actions_t'])

                out['target_state'] = target_state
                out['dtime'] = dt
                out['final_time'] = final_time
                out['time'] = traj_cvx_i['time'][:-1]
                out['obstacles'] = obs
            else:
                out['feasible'] = False
        except:
            out['feasible'] = False
    else:
        out['feasible'] = False
    
    return out

if __name__ == '__main__':

    N_data = 200000
    set_start_method('spawn')

    n_S = N_STATE  # state size
    n_A = N_ACTION  # action size
    n_C = N_CLUSTERS  # cluster size
    n_obs_max = N_OBS_MAX  # maximum number of obstacles

    # Model initialization
    ff_model = FreeflyerModel()
    other_args = {
        'ff_model' : ff_model,
        'scenario_rand' : 'list'
    }

    if generalized_time:
        dataset_scenario_folder = '/' + dataset_scenario

        states_cvx = np.empty(shape=(N_data, ), dtype=object) # [m,m,m,m/s,m/s,m/s]
        actions_cvx = np.empty(shape=(N_data, ), dtype=object) # [m/s]
        actions_t_cvx = np.empty(shape=(N_data, ), dtype=object)

        states_scp = np.empty(shape=(N_data, ), dtype=object) # [m,m,m,m/s,m/s,m/s]
        actions_scp = np.empty(shape=(N_data, ), dtype=object) # [m/s]
        actions_t_scp = np.empty(shape=(N_data, ), dtype=object)

        target_state = np.empty(shape=(N_data, n_S), dtype=float)
        dtime = np.empty(shape=(N_data, ), dtype=float)
        final_time = np.empty(shape=(N_data, ), dtype=float)
        time = np.empty(shape=(N_data, ), dtype=object)
    else:
        dataset_scenario_folder = '' if (not generalized_obs) else ('/' + dataset_scenario)
        n_time_rpod = round(T_nominal/dt)

        states_cvx = np.empty(shape=(N_data, n_time_rpod, n_S), dtype=float) # [m,m,m,m/s,m/s,m/s]
        actions_cvx = np.empty(shape=(N_data, n_time_rpod, n_A), dtype=float) # [m/s]
        actions_t_cvx = np.empty(shape=(N_data, n_time_rpod, n_C), dtype=float)

        states_scp = np.empty(shape=(N_data, n_time_rpod, n_S), dtype=float) # [m,m,m,m/s,m/s,m/s]
        actions_scp = np.empty(shape=(N_data, n_time_rpod, n_A), dtype=float) # [m/s]
        actions_t_scp = np.empty(shape=(N_data, n_time_rpod, n_C), dtype=float)

        target_state = np.empty(shape=(N_data, n_S), dtype=float)
        dtime = np.empty(shape=(N_data, ), dtype=float)
        time = np.empty(shape=(N_data, n_time_rpod), dtype=float)

        if generalized_obs:
            n_obs = - np.ones(shape=(N_data,), dtype=int)
            obs_position = np.empty(shape=(N_data, n_obs_max, 2), dtype=float)
            obs_radius = np.empty(shape=(N_data, n_obs_max), dtype=float)
            # For other types of observation (such as a 2d map), the following line should be replaced
            observations = np.zeros(shape=(N_data, n_time_rpod, 3 * n_obs_max), dtype=float)

    i_unfeas = []

    # Pool creation --> Should automatically select the maximum number of processes
    p = Pool(processes=24)
    for i, res in enumerate(tqdm(p.imap(for_computation, zip(np.arange(N_data), itertools.repeat(other_args))), total=N_data)):
    #for i in np.arange(N_data):
    #    res = for_computation((i, other_args))
        # If the solution is feasible save the optimization output
        if res['feasible']:
            states_cvx[i] = res['states_cvx']
            actions_cvx[i] = res['actions_cvx']
            actions_t_cvx[i] = res['actions_t_cvx']

            states_scp[i] = res['states_scp']
            actions_scp[i] = res['actions_scp']
            actions_t_scp[i] = res['actions_t_scp']

            target_state[i,:] = res['target_state']
            dtime[i] = res['dtime']
            if generalized_time:
                final_time[i] = res['final_time']
            time[i] = res['time']
            if generalized_obs:
                n_obs[i] = len(res['obstacles']['radius'])
                obs_position[i, :n_obs[i]] = res['obstacles']['position']
                obs_radius[i, :n_obs[i]] = res['obstacles']['radius']

                # For other types of observation (such as a 2d map), the following line should be replaced
                observations[i, :, :3 * n_obs[i]] = generate_perfect_observations(res['obstacles']['position'],
                                                                                  res['obstacles']['radius'])

        # Else add the index to the list
        else:
            i_unfeas += [ i ]
        
        if i % 50000 == 0:
            np.savez_compressed(root_folder + '/dataset' + dataset_scenario_folder + '/dataset-ff-v05-scp' + str(i), states_scp = states_scp, actions_scp = actions_scp, actions_t_scp = actions_t_scp, i_unfeas = i_unfeas)
            np.savez_compressed(root_folder + '/dataset' + dataset_scenario_folder + '/dataset-ff-v05-cvx' + str(i), states_cvx = states_cvx, actions_cvx = actions_cvx, actions_t_cvx = actions_t_cvx, i_unfeas = i_unfeas)
            if generalized_time:
                np.savez_compressed(root_folder + '/dataset' + dataset_scenario_folder + '/dataset-ff-v05-param' + str(i), target_state = target_state, time = time, final_time = final_time, dtime = dtime, i_unfeas = i_unfeas)
            elif generalized_time:
                np.savez_compressed(root_folder + '/dataset' + dataset_scenario_folder + '/dataset-ff-v05-param' + str(i), target_state = target_state, time = time, dtime = dtime, n_obs = n_obs, obs_position = obs_position, obs_radius = obs_radius, i_unfeas = i_unfeas)
            else:
                np.savez_compressed(root_folder + '/dataset' + dataset_scenario_folder + '/dataset-ff-v05-param' + str(i), target_state = target_state, time = time, dtime = dtime, i_unfeas = i_unfeas)
    
    # Remove unfeasible data points
    if i_unfeas:
        states_cvx = np.delete(states_cvx, i_unfeas, axis=0)
        actions_cvx = np.delete(actions_cvx, i_unfeas, axis=0)
        actions_t_cvx = np.delete(actions_t_cvx, i_unfeas, axis=0)

        states_scp = np.delete(states_scp, i_unfeas, axis=0)
        actions_scp = np.delete(actions_scp, i_unfeas, axis=0)
        actions_t_scp = np.delete(actions_t_scp, i_unfeas, axis=0)

        target_state = np.delete(target_state, i_unfeas, axis=0)
        dtime = np.delete(dtime, i_unfeas, axis=0)
        if generalized_time:
            final_time = np.delete(final_time, i_unfeas, axis=0)
        time = np.delete(time, i_unfeas, axis=0)
        if generalized_obs:
            n_obs = np.delete(n_obs, i_unfeas, axis=0)
            obs_position = np.delete(obs_position, i_unfeas, axis=0)
            obs_radius = np.delete(obs_radius, i_unfeas, axis=0)
            observations = np.delete(observations, i_unfeas, axis=0)

    #  Save dataset (local folder for the workstation)
    np.savez_compressed(root_folder + '/dataset' + dataset_scenario_folder + '/dataset-ff-v05-scp', states_scp = states_scp, actions_scp = actions_scp, actions_t_scp = actions_t_scp)
    np.savez_compressed(root_folder + '/dataset' + dataset_scenario_folder + '/dataset-ff-v05-cvx', states_cvx = states_cvx, actions_cvx = actions_cvx, actions_t_cvx = actions_t_cvx)
    if generalized_time:
        np.savez_compressed(root_folder + '/dataset' + dataset_scenario_folder + '/dataset-ff-v05-param', target_state = target_state, time = time, final_time=final_time, dtime = dtime)
    elif generalized_obs:
        np.savez_compressed(root_folder + '/dataset' + dataset_scenario_folder + '/dataset-ff-v05-param', target_state = target_state, time = time, dtime = dtime, n_obs = n_obs, obs_position = obs_position, obs_radius = obs_radius)
    else:
        np.savez_compressed(root_folder + '/dataset' + dataset_scenario_folder + '/dataset-ff-v05-param', target_state = target_state, time = time, dtime = dtime)
