import os
import sys

root_folder = os.path.abspath(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

sys.path.append(root_folder)

import numpy as np
import numpy.linalg as la
import torch

import decision_transformer.manage as DT_manager
from dynamics.freeflyer import FreeflyerModel, ocp_no_obstacle_avoidance, ocp_obstacle_avoidance, compute_constraint_to_go, sample_init_target
from optimization.ff_scenario import N_STATE, N_ACTION, T_min, T_max, n_time_max, obs, iter_max_SCP, robot_radius, safety_margin, random_chunk
import time
import itertools
from multiprocessing import Pool, set_start_method
from tqdm import tqdm

def for_computation(input_iterable):

    # Extract input
    current_idx = input_iterable[0]
    input_dict = input_iterable[1]
    model = input_dict['model']
    model2 = input_dict['model2']
    model3 = input_dict['model3']
    test_loader = input_dict['test_loader']
    test_loader2 = input_dict['test_loader2']
    test_loader3 = input_dict['test_loader3']
    transformer_ws = input_dict['transformer_ws']
    mdp_constr = input_dict['mdp_constr']
    sample_init_final = input_dict['sample_init_final']
    final_time = input_dict['final_time']
    chunksize = input_dict['chunksize']

    # Output dictionary initialization
    out = {'feasible_cvx' : True,
           'feasible_scp_cvx' : True,
           'feasible_scp_DT3' : True,
           'feasible_scp_DT2' : True,
           'feasible_scp_DT' : True,
           'J_vect_scp_cvx': [],
           'J_vect_scp_DT3' : [],
           'J_vect_scp_DT2' : [],
           'J_vect_scp_DT': [],
           'J_cvx' : [],
           'J_DT' : [],
           'J_DT2' : [],
           'J_DT3' : [],
           'iter_scp_cvx': [],
           'iter_scp_DT3' : [],
           'iter_scp_DT2' : [],
           'iter_scp_DT': [],
           'runtime_cvx': [],
           'runtime_DT': [],
           'runtime_DT2' : [],
           'runtime_DT3' : [],
           'runtime_scp_cvx': [],
           'runtime_scp_DT3' : [],
           'runtime_scp_DT2' : [],
           'runtime_scp_DT': [],
           'ctgs0_cvx': [],
           'cvx_problem' : False,
           'test_dataset_ix' : [],
           'state_init' : [],
           'state_final' : []
          }
   
    if sample_init_final:
        state_init, state_final = sample_init_target(sample_time=False)
        test_sample = DT_manager.get_fake_sample_like(test_loader, state_init, state_final, final_time)
        test_sample2 = DT_manager.get_fake_sample_like(test_loader2, state_init, state_final, final_time)
        test_sample3 = DT_manager.get_fake_sample_like(test_loader3, state_init, state_final, final_time)
        '''test_sample[0][0,:,:] = (torch.tensor(np.repeat(state_init[None,:], n_time_max, axis=0)) - data_stats['states_mean'])/(data_stats['states_std'] + 1e-6)
        test_sample[1][0,:,:] = torch.zeros((n_time_max,N_ACTION))
        test_sample[2][0,:,0] = torch.zeros((n_time_max,))
        test_sample[3][0,:,0] = torch.zeros((n_time_max,))
        test_sample[4][0,:,0] = torch.zeros((n_time_max,))
        test_sample[5][0,:,:] = (torch.tensor(np.repeat(state_final[None,:], n_time_max, axis=0)) - data_stats['goal_mean'])/(data_stats['goal_std'] + 1e-6)'''
        out['test_dataset_ix'] = test_sample[-1].item()
    else:
        test_sample = test_loader.dataset.getix(current_idx)
        data_stats = test_loader.dataset.data_stats
        if not mdp_constr:
            states_i, actions_i, rtgs_i, goal_i, timesteps_i, attention_mask_i, dt, time_sec, ix = test_sample
        else:
            states_i, actions_i, rtgs_i, ctgs_i, ttgs_i, goal_i, timesteps_i, attention_mask_i, dt, time_sec, ix = test_sample
        # print('Sampled trajectory ' + str(ix) + ' from test_dataset.')
        out['test_dataset_ix'] = ix.item()
        state_init = np.array((states_i[0, 0, :] * data_stats['states_std'][0]) + data_stats['states_mean'][0])
        state_final = np.array((goal_i[0, 0, :] * data_stats['goal_std'][0]) + data_stats['goal_mean'][0])

    out['state_init'] = state_init
    out['state_final'] = state_final
    ff_model = FreeflyerModel()


    ####### Warmstart Convex Problem RPOD
    try:
        runtime0_cvx = time.time()
        traj_cvx, _, _, feas_cvx = ocp_no_obstacle_avoidance(ff_model, state_init, state_final, final_time)
        runtime1_cvx = time.time()
        runtime_cvx = runtime1_cvx-runtime0_cvx
        states_cvx, actions_cvx = traj_cvx['states'], traj_cvx['actions_G']
    except:
        states_cvx = None
        actions_cvx = None
        feas_cvx = 'failure'
        runtime_cvx = None
    
    if np.char.equal(feas_cvx,'optimal'):
        states_ws_cvx = states_cvx # set warm start
        actions_ws_cvx = actions_cvx # set warm start
        out['J_cvx'] = np.sum(la.norm(actions_ws_cvx, ord=1, axis=0))
        # Evaluate Constraint Violation
        ctgs_cvx = compute_constraint_to_go(states_ws_cvx.T, obs['position'], (obs['radius'] + robot_radius)*safety_margin)
        ctgs0_cvx = ctgs_cvx[0,0]
        # Save cvx in the output dictionary
        out['runtime_cvx'] = runtime_cvx
        out['ctgs0_cvx'] = ctgs0_cvx
        out['cvx_problem'] = ctgs0_cvx == 0

        # Solve SCP
        runtime0_scp_cvx = time.time()
        traj_scp_cvx, J_vect_scp_cvx, iter_scp_cvx, feas_scp_cvx = ocp_obstacle_avoidance(ff_model, states_ws_cvx, actions_ws_cvx, state_init, state_final)
        runtime1_scp_cvx = time.time()
        runtime_scp_cvx = runtime1_scp_cvx - runtime0_scp_cvx
        
        if np.char.equal(feas_scp_cvx,'optimal'):
            # Save scp_cvx data in the output dictionary
            out['J_vect_scp_cvx'] = J_vect_scp_cvx
            out['iter_scp_cvx'] = iter_scp_cvx    
            out['runtime_scp_cvx'] = runtime_scp_cvx
        else:
            out['feasible_scp_cvx'] = False
    else:
        out['feasible_scp_cvx'] = False
        out['feasible_cvx'] = False

    ####### Warmstart Transformer
    # Import the Transformer
    if np.char.equal(feas_cvx,'optimal'):
        if mdp_constr:
            rtg_0 = -out['J_cvx']
            if transformer_ws == 'dyn':
                DT_trajectory, runtime_DT = DT_manager.torch_model_inference_dyn(model, test_loader, test_sample, rtg_perc=None, ctg_perc=0., rtg=rtg_0,
                                                                                ttg=final_time, ctg_clipped=True, chunksize=chunksize, end_on_ttg=True)
            elif transformer_ws == 'ol':
                DT_trajectory, runtime_DT = DT_manager.torch_model_inference_ol(model, test_loader, test_sample, rtg_perc=None, ctg_perc=0., rtg=rtg_0,
                                                                                ttg=final_time, ctg_clipped=True, chunksize=chunksize, end_on_ttg=True)
        else:
            if transformer_ws == 'dyn':
                DT_trajectory, runtime_DT = DT_manager.torch_model_inference_dyn(model, test_loader, test_sample, rtg_perc=1., ctg_perc=0., rtg=None,
                                                                                ttg=final_time, ctg_clipped=True, chunksize=chunksize, end_on_ttg=True)
            elif transformer_ws == 'ol':
                DT_trajectory, runtime_DT = DT_manager.torch_model_inference_ol(model, test_loader, test_sample, rtg_perc=1., ctg_perc=0., rtg=None,
                                                                                ttg=final_time, ctg_clipped=True, chunksize=chunksize, end_on_ttg=True)    
        out['J_DT'] = np.sum(la.norm(DT_trajectory['dv_' + transformer_ws], ord=1, axis=0))
        states_ws_DT = np.hstack((DT_trajectory['xypsi_' + transformer_ws],
                                  (DT_trajectory['xypsi_' + transformer_ws][:,-1] + ff_model.B_imp @ DT_trajectory['dv_' + transformer_ws][:, -1]).reshape((6,1)))) # set warm start
        actions_ws_DT = DT_trajectory['dv_' + transformer_ws] # set warm start
        # Save DT in the output dictionary
        out['runtime_DT'] = runtime_DT

        # Solve SCP
        runtime0_scp_DT = time.time()
        traj_scp_DT, J_vect_scp_DT, iter_scp_DT, feas_scp_DT = ocp_obstacle_avoidance(ff_model, states_ws_DT, actions_ws_DT, state_init, state_final)
        runtime1_scp_DT = time.time()
        runtime_scp_DT = runtime1_scp_DT - runtime0_scp_DT
        
        if np.char.equal(feas_scp_DT,'optimal'):
            # Save scp_DT in the output dictionary
            out['J_vect_scp_DT'] = J_vect_scp_DT
            out['iter_scp_DT'] = iter_scp_DT
            out['runtime_scp_DT'] = runtime_scp_DT
        else:
            out['feasible_scp_DT'] = False
    else:
        out['feasible_scp_DT'] = False

    if np.char.equal(feas_cvx,'optimal') and (not (model2 is None)):
        # Import the Transformer2
        if mdp_constr:
            if transformer_ws == 'dyn':
                DT2_trajectory, runtime_DT2 = DT_manager.torch_model_inference_dyn(model2, test_loader2, test_sample2, rtg_perc=None, ctg_perc=0., rtg=rtg_0,
                                                                                   ttg=final_time, ctg_clipped=True, chunksize=chunksize, end_on_ttg=True)
            elif transformer_ws == 'ol':
                DT2_trajectory, runtime_DT2 = DT_manager.torch_model_inference_ol(model2, test_loader2, test_sample2, rtg_perc=None, ctg_perc=0., rtg=rtg_0,
                                                                                  ttg=final_time, ctg_clipped=True, chunksize=chunksize, end_on_ttg=True)
        else:
            if transformer_ws == 'dyn':
                DT2_trajectory, runtime_DT2 = DT_manager.torch_model_inference_dyn(model2, test_loader2, test_sample2, rtg_perc=1., ctg_perc=0., rtg=None,
                                                                                   ttg=final_time, ctg_clipped=True, chunksize=chunksize, end_on_ttg=True)
            elif transformer_ws == 'ol':
                DT2_trajectory, runtime_DT2 = DT_manager.torch_model_inference_ol(model2, test_loader2, test_sample2, rtg_perc=1., ctg_perc=0., rtg=None,
                                                                                  ttg=final_time, ctg_clipped=True, chunksize=chunksize, end_on_ttg=True)    
        out['J_DT2'] = np.sum(la.norm(DT2_trajectory['dv_' + transformer_ws], ord=1, axis=0))
        states_ws_DT2 = np.hstack((DT2_trajectory['xypsi_' + transformer_ws],
                                   (DT2_trajectory['xypsi_' + transformer_ws][:,-1] + ff_model.B_imp @ DT2_trajectory['dv_' + transformer_ws][:, -1]).reshape((6,1)))) # set warm start
        actions_ws_DT2 = DT2_trajectory['dv_' + transformer_ws] # set warm start
        # Save DT in the output dictionary
        out['runtime_DT2'] = runtime_DT2

        # Solve SCP
        runtime0_scp_DT2 = time.time()
        traj_scp_DT2, J_vect_scp_DT2, iter_scp_DT2, feas_scp_DT2 = ocp_obstacle_avoidance(ff_model, states_ws_DT2, actions_ws_DT2, state_init, state_final)
        runtime1_scp_DT2 = time.time()
        runtime_scp_DT2 = runtime1_scp_DT2 - runtime0_scp_DT2
        
        if np.char.equal(feas_scp_DT2,'optimal'):
            # Save scp_DT in the output dictionary
            out['J_vect_scp_DT2'] = J_vect_scp_DT2
            out['iter_scp_DT2'] = iter_scp_DT2
            out['runtime_scp_DT2'] = runtime_scp_DT2
        else:
            out['feasible_scp_DT2'] = False
    else:
        out['J_DT2'] = -1.
        out['runtime_DT2'] = -1.
        out['feasible_scp_DT2'] = False
    
    if np.char.equal(feas_cvx,'optimal') and (not (model3 is None)):
        # Import the Transformer2
        if mdp_constr:
            if transformer_ws == 'dyn':
                DT3_trajectory, runtime_DT3 = DT_manager.torch_model_inference_dyn(model3, test_loader3, test_sample3, rtg_perc=None, ctg_perc=0., rtg=rtg_0,
                                                                                   ttg=final_time, ctg_clipped=True, chunksize=chunksize, end_on_ttg=True)
            elif transformer_ws == 'ol':
                DT3_trajectory, runtime_DT3 = DT_manager.torch_model_inference_ol(model3, test_loader3, test_sample3, rtg_perc=None, ctg_perc=0., rtg=rtg_0,
                                                                                  ttg=final_time, ctg_clipped=True, chunksize=chunksize, end_on_ttg=True)
        else:
            if transformer_ws == 'dyn':
                DT3_trajectory, runtime_DT3 = DT_manager.torch_model_inference_dyn(model3, test_loader3, test_sample3, rtg_perc=1., ctg_perc=0., rtg=None,
                                                                                   ttg=final_time, ctg_clipped=True, chunksize=chunksize, end_on_ttg=True)
            elif transformer_ws == 'ol':
                DT3_trajectory, runtime_DT3 = DT_manager.torch_model_inference_ol(model3, test_loader3, test_sample3, rtg_perc=1., ctg_perc=0., rtg=None,
                                                                                  ttg=final_time, ctg_clipped=True, chunksize=chunksize, end_on_ttg=True)    
        out['J_DT3'] = np.sum(la.norm(DT3_trajectory['dv_' + transformer_ws], ord=1, axis=0))
        states_ws_DT3 = np.hstack((DT3_trajectory['xypsi_' + transformer_ws],
                                   (DT3_trajectory['xypsi_' + transformer_ws][:,-1] + ff_model.B_imp @ DT3_trajectory['dv_' + transformer_ws][:, -1]).reshape((6,1)))) # set warm start
        actions_ws_DT3 = DT3_trajectory['dv_' + transformer_ws] # set warm start
        # Save DT in the output dictionary
        out['runtime_DT3'] = runtime_DT3

        # Solve SCP
        runtime0_scp_DT3 = time.time()
        traj_scp_DT3, J_vect_scp_DT3, iter_scp_DT3, feas_scp_DT3 = ocp_obstacle_avoidance(ff_model, states_ws_DT3, actions_ws_DT3, state_init, state_final)
        runtime1_scp_DT3 = time.time()
        runtime_scp_DT3 = runtime1_scp_DT3 - runtime0_scp_DT3
        
        if np.char.equal(feas_scp_DT3,'optimal'):
            # Save scp_DT in the output dictionary
            out['J_vect_scp_DT3'] = J_vect_scp_DT3
            out['iter_scp_DT3'] = iter_scp_DT3
            out['runtime_scp_DT3'] = runtime_scp_DT3
        else:
            out['feasible_scp_DT3'] = False
    else:
        out['J_DT3'] = -1.
        out['runtime_DT3'] = -1.
        out['feasible_scp_DT3'] = False

    return out

if __name__ == '__main__':

    transformer_ws = 'dyn' # 'dyn'/'ol'
    transformer_model_name = 'checkpoint_ff_time40_100_chunk100R_ctgrtg12000'
    transformer_model_name2 = 'checkpoint_ff_time_chunk100R_ctgrtg17000'
    transformer_model_name3 = 'checkpoint_ff_time_const90_chunk100R_ctgrtg'
    import_config = DT_manager.transformer_import_config(transformer_model_name)
    import_config['dataset_scenario'] = 'time_40_100'
    import_config2 = DT_manager.transformer_import_config(transformer_model_name2)
    import_config2['dataset_scenario'] = 'time'
    import_config3 = DT_manager.transformer_import_config(transformer_model_name3)
    import_config3['dataset_scenario'] = 'time_const_90'
    set_start_method('spawn')
    num_processes = 10

    # Get the datasets and loaders from the torch data
    datasets, dataloaders = DT_manager.get_train_val_test_data(mdp_constr=import_config['mdp_constr'], dataset_scenario=import_config['dataset_scenario'],
                                                               timestep_norm=import_config['timestep_norm'], chunksize=import_config['chunksize'], random_chunk=random_chunk)
    train_loader, eval_loader, test_loader = dataloaders
    model = DT_manager.get_DT_model(transformer_model_name, train_loader, eval_loader)
    if not(transformer_model_name2 is None):
        datasets2, dataloaders2 = DT_manager.get_train_val_test_data(mdp_constr=import_config2['mdp_constr'], dataset_scenario=import_config2['dataset_scenario'],
                                                                     timestep_norm=import_config2['timestep_norm'], chunksize=import_config2['chunksize'], random_chunk=random_chunk)
        train_loader2, eval_loader2, test_loader2 = dataloaders2
        model2 = DT_manager.get_DT_model(transformer_model_name2, train_loader2, eval_loader2)
    else:
        test_loader2 = None
        model2 = None
    if not(transformer_model_name3 is None):
        datasets3, dataloaders3 = DT_manager.get_train_val_test_data(mdp_constr=import_config3['mdp_constr'], dataset_scenario=import_config3['dataset_scenario'],
                                                                     timestep_norm=import_config3['timestep_norm'], chunksize=import_config3['chunksize'], random_chunk=random_chunk)
        train_loader3, eval_loader3, test_loader3 = dataloaders3
        model3 = DT_manager.get_DT_model(transformer_model_name3, train_loader3, eval_loader3)
    else:
        test_loader3 = None
        model3 = None

    # List of possible time allotted for the trajectory
    ttg_com_list = np.arange(T_min, T_max+1, 10.0)

    for ttg_com in ttg_com_list:
        # Parallel for inputs
        print("############### Beginning analysis for final_time =", ttg_com, "############################")
        N_data_test = 5000#test_loader.dataset.n_data
        other_args = {
            'model' : model,
            'model2' : model2,
            'model3' : model3,
            'test_loader' : test_loader,
            'test_loader2' : test_loader2,
            'test_loader3' : test_loader3,
            'transformer_ws' : transformer_ws,
            'mdp_constr' : import_config['mdp_constr'],
            'final_time' : ttg_com,
            'chunksize' : import_config['chunksize'],
            'sample_init_final' : True
        }
        print('Sample_init_final =', other_args['sample_init_final'])

        J_vect_scp_cvx = np.empty(shape=(N_data_test, iter_max_SCP), dtype=float)
        J_vect_scp_DT3 = np.empty(shape=(N_data_test, iter_max_SCP), dtype=float)
        J_vect_scp_DT2 = np.empty(shape=(N_data_test, iter_max_SCP), dtype=float)
        J_vect_scp_DT = np.empty(shape=(N_data_test, iter_max_SCP), dtype=float)
        J_cvx = np.empty(shape=(N_data_test, ), dtype=float)
        J_DT = np.empty(shape=(N_data_test, ), dtype=float)
        J_DT2 = np.empty(shape=(N_data_test, ), dtype=float)
        J_DT3 = np.empty(shape=(N_data_test, ), dtype=float)
        iter_scp_cvx = np.empty(shape=(N_data_test, ), dtype=float)
        iter_scp_DT3 = np.empty(shape=(N_data_test, ), dtype=float)
        iter_scp_DT2 = np.empty(shape=(N_data_test, ), dtype=float) 
        iter_scp_DT = np.empty(shape=(N_data_test, ), dtype=float) 
        runtime_cvx = np.empty(shape=(N_data_test, ), dtype=float) 
        runtime_DT = np.empty(shape=(N_data_test, ), dtype=float)
        runtime_DT2 = np.empty(shape=(N_data_test, ), dtype=float)
        runtime_DT3 = np.empty(shape=(N_data_test, ), dtype=float)
        runtime_scp_cvx = np.empty(shape=(N_data_test, ), dtype=float)
        runtime_scp_DT3 = np.empty(shape=(N_data_test, ), dtype=float) 
        runtime_scp_DT2 = np.empty(shape=(N_data_test, ), dtype=float)
        runtime_scp_DT = np.empty(shape=(N_data_test, ), dtype=float) 
        ctgs0_cvx = np.empty(shape=(N_data_test, ), dtype=float)
        cvx_problem = np.full(shape=(N_data_test, ), fill_value=False)
        test_dataset_ix = np.empty(shape=(N_data_test, ), dtype=float)
        state_init = np.empty(shape=(N_data_test, N_STATE), dtype=float)
        state_final = np.empty(shape=(N_data_test, N_STATE), dtype=float)

        i_unfeas_cvx = []
        i_unfeas_scp_cvx = []
        i_unfeas_scp_DT3 = []
        i_unfeas_scp_DT2 = []
        i_unfeas_scp_DT = []

        # Pool creation --> Should automatically select the maximum number of processes
        p = Pool(processes=num_processes)
        for i, res in enumerate(tqdm(p.imap(for_computation, zip(np.arange(N_data_test), itertools.repeat(other_args))), total=N_data_test)):
            #for i in np.arange(N_data_test):
            #res = for_computation((i, other_args))

            # Save the input in the dataset
            test_dataset_ix[i] = res['test_dataset_ix']
            state_init[i] = res['state_init']
            state_final[i] = res['state_final']

            # If the solution is feasible save the optimization output
            if res['feasible_cvx']:
                J_cvx[i] = res['J_cvx']
                runtime_cvx[i] = res['runtime_cvx']
                ctgs0_cvx[i] = res['ctgs0_cvx']
                cvx_problem[i] = res['cvx_problem']
                J_DT[i] = res['J_DT']
                runtime_DT[i] = res['runtime_DT']
                J_DT2[i] = res['J_DT2']
                runtime_DT2[i] = res['runtime_DT2']
                J_DT3[i] = res['J_DT3']
                runtime_DT3[i] = res['runtime_DT3']
            else:
                i_unfeas_cvx += [ i ]

            if res['feasible_scp_cvx']:
                J_vect_scp_cvx[i,:] = res['J_vect_scp_cvx']
                iter_scp_cvx[i] = res['iter_scp_cvx']
                runtime_scp_cvx[i] = res['runtime_scp_cvx']
            else:
                i_unfeas_scp_cvx += [ i ]

            if res['feasible_scp_DT']:
                J_vect_scp_DT[i,:] = res['J_vect_scp_DT']
                iter_scp_DT[i] = res['iter_scp_DT']
                runtime_scp_DT[i] = res['runtime_scp_DT']
            else:
                i_unfeas_scp_DT += [ i ]
            
            if res['feasible_scp_DT2']:
                J_vect_scp_DT2[i,:] = res['J_vect_scp_DT2']
                iter_scp_DT2[i] = res['iter_scp_DT2']
                runtime_scp_DT2[i] = res['runtime_scp_DT2']
            else:
                i_unfeas_scp_DT2 += [ i ]
            
            if res['feasible_scp_DT3']:
                J_vect_scp_DT3[i,:] = res['J_vect_scp_DT3']
                iter_scp_DT3[i] = res['iter_scp_DT3']
                runtime_scp_DT3[i] = res['runtime_scp_DT3']
            else:
                i_unfeas_scp_DT3 += [ i ]
            
            if i % 10000 == 0:
                #  Save dataset (local folder for the workstation)
                np.savez_compressed(root_folder + '/optimization/saved_files/warmstarting/ws_analysis_' + transformer_model_name + '_comp_' + transformer_ws + str(ttg_com) + '_'+ str(i),
                                    model_name = transformer_model_name,
                                    model_name2 = transformer_model_name2,
                                    model_name3 = transformer_model_name3,
                                    J_vect_scp_cvx = J_vect_scp_cvx,
                                    J_vect_scp_DT3 = J_vect_scp_DT3,
                                    J_vect_scp_DT2 = J_vect_scp_DT2,
                                    J_vect_scp_DT = J_vect_scp_DT,
                                    J_cvx = J_cvx,
                                    J_DT = J_DT,
                                    J_DT2 = J_DT2,
                                    J_DT3 = J_DT3,
                                    iter_scp_cvx = iter_scp_cvx,
                                    iter_scp_DT3 = iter_scp_DT3,
                                    iter_scp_DT2 = iter_scp_DT2,
                                    iter_scp_DT = iter_scp_DT,
                                    runtime_cvx = runtime_cvx,
                                    runtime_DT = runtime_DT,
                                    runtime_DT2 = runtime_DT2,
                                    runtime_DT3 = runtime_DT3,
                                    runtime_scp_cvx = runtime_scp_cvx,
                                    runtime_scp_DT3 = runtime_scp_DT3,
                                    runtime_scp_DT2 = runtime_scp_DT2,
                                    runtime_scp_DT = runtime_scp_DT,
                                    ctgs0_cvx = ctgs0_cvx, 
                                    cvx_problem = cvx_problem,
                                    test_dataset_ix = test_dataset_ix,
                                    state_init = state_init,
                                    state_final = state_final,
                                    i_unfeas_cvx = i_unfeas_cvx,
                                    i_unfeas_scp_cvx = i_unfeas_scp_cvx,
                                    i_unfeas_scp_DT3 = i_unfeas_scp_DT3,
                                    i_unfeas_scp_DT2 = i_unfeas_scp_DT2,
                                    i_unfeas_scp_DT = i_unfeas_scp_DT,
                                    final_time = ttg_com,
                                    )

        
        #  Save dataset (local folder for the workstation)
        np.savez_compressed(root_folder + '/optimization/saved_files/warmstarting/ws_analysis_' + transformer_model_name + '_comp_' + transformer_ws + str(ttg_com),
                            model_name = transformer_model_name,
                            model_name2 = transformer_model_name2,
                            model_name3 = transformer_model_name3,
                            J_vect_scp_cvx = J_vect_scp_cvx,
                            J_vect_scp_DT3 = J_vect_scp_DT3,
                            J_vect_scp_DT2 = J_vect_scp_DT2,
                            J_vect_scp_DT = J_vect_scp_DT,
                            J_cvx = J_cvx,
                            J_DT = J_DT,
                            J_DT2 = J_DT2,
                            J_DT3 = J_DT3,
                            iter_scp_cvx = iter_scp_cvx,
                            iter_scp_DT3 = iter_scp_DT3,
                            iter_scp_DT2 = iter_scp_DT2,
                            iter_scp_DT = iter_scp_DT,
                            runtime_cvx = runtime_cvx,
                            runtime_DT = runtime_DT,
                            runtime_DT2 = runtime_DT2,
                            runtime_DT3 = runtime_DT3,
                            runtime_scp_cvx = runtime_scp_cvx,
                            runtime_scp_DT3 = runtime_scp_DT3,
                            runtime_scp_DT2 = runtime_scp_DT2,
                            runtime_scp_DT = runtime_scp_DT,
                            ctgs0_cvx = ctgs0_cvx, 
                            cvx_problem = cvx_problem,
                            test_dataset_ix = test_dataset_ix,
                            state_init = state_init,
                            state_final = state_final,
                            i_unfeas_cvx = i_unfeas_cvx,
                            i_unfeas_scp_cvx = i_unfeas_scp_cvx,
                            i_unfeas_scp_DT3 = i_unfeas_scp_DT3,
                            i_unfeas_scp_DT2 = i_unfeas_scp_DT2,
                            i_unfeas_scp_DT = i_unfeas_scp_DT,
                            final_time = ttg_com
                            )