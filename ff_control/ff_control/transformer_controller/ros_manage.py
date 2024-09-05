import os
import sys
root_folder = os.path.dirname(os.path.abspath(__file__))
sys.path.append(root_folder)
import numpy as np
import copy

import torch
from transformers import DecisionTransformerConfig
from accelerate import Accelerator
from ff_control.transformer_controller.art import AutonomousFreeflyerTransformer, AutonomousFreeflyerTransformer_pred_time
from ff_control.transformer_controller.freeflyer import FreeflyerModel
from ff_control.transformer_controller.ff_scenario import generalized_time, generalized_obs, obs, safety_margin, robot_radius, dt, n_time_max, T_nominal
import time
# select device based on availability of GPU
verbose = False # set to True to get additional print statements
device = torch.device('cuda') if torch.cuda.is_available() else torch.device('cpu')
print(device)

'''
    TODO: 
        
'''

def get_DT_model(model_name, n_state, n_action):
    # DT model creation
    config = DecisionTransformerConfig(
        state_dim=n_state, 
        act_dim=n_action,
        hidden_size=384,
        max_ep_len=n_time_max,
        vocab_size=1,
        action_tanh=False,
        n_positions=1024 if (not generalized_time) else 2048,
        n_layer=6,
        n_head=6,
        n_inner=None,
        resid_pdrop=0.1,
        embd_pdrop=0.1,
        attn_pdrop=0.1,
        )
    if generalized_time:
        model = AutonomousFreeflyerTransformer_pred_time(config)
    else:
        model = AutonomousFreeflyerTransformer(config)
    model_size = sum(t.numel() for t in model.parameters())
    print(f"GPT size: {model_size/1000**2:.1f}M parameters")
    model.to(device);

    # DT optimizer and accelerator
    accelerator = Accelerator(mixed_precision='no', gradient_accumulation_steps=8)
    model = accelerator.prepare(model)
    accelerator.load_state(root_folder+'/saved_files/checkpoints/' + model_name)

    return model.eval()

def get_data_stats(model_name):
    return np.load(root_folder+'/saved_files/checkpoints/' + model_name +'/data_stats.npz', allow_pickle=True)['data_stats'].item()

def state_init_final2sample(data_stats, state_init, state_final, final_time=T_nominal):
    '''
    Method to create a fake sample from desired initial/final state and final time. The fake sample has the sample structure of a real random sample taken from dataloader can be used to initialize the trajectory generation process at inference.
    '''
    n_time = data_stats['states_mean'].shape[0]
    ix = torch.tensor([0])[None,:]
    ttg = torch.arange(final_time, 0, -dt).to(device)
    ttg_sample = torch.zeros((n_time_max,), device=device)
    ttg_sample[:ttg.shape[0]] = ttg
    states_i = ((torch.tensor(np.repeat(state_init[None,:], n_time_max, axis=0), device=device) - data_stats['states_mean'])/(data_stats['states_std'] + 1e-6))[None,:,:].float()
    actions_i = torch.zeros((n_time_max,3))[None,:,:].float()
    rtgs_i = torch.zeros((n_time_max,))[None,:,None].float()
    ctgs_i = torch.zeros((n_time_max,))[None,:,None].float()
    goal_i = ((torch.tensor(np.repeat(state_final[None,:], n_time_max, axis=0), device=device) - data_stats['goal_mean'])/(data_stats['goal_std'] + 1e-6))[None,:,:].float()
    timesteps_i = (torch.tensor([[i for i in range(n_time_max)] for _ in ix]).view(n_time_max).long())[None,:]
    attention_mask_i = (torch.ones(1, n_time_max).view(n_time_max).long())[None,:]
    dt_i = torch.tensor([dt]).float()
    time_sec_i = torch.arange(0, n_time_max*dt, dt).float()[None,None,:]
    
    if generalized_time:
        ttgs_i = ((ttg_sample[:,None] - data_stats['ttgs_mean'])/(data_stats['ttgs_std'] + 1e-6))[None,:,:].float()
        return states_i, actions_i, rtgs_i, ctgs_i, ttgs_i, goal_i, timesteps_i, attention_mask_i, dt_i, time_sec_i, ix
    else:
        return states_i, actions_i, rtgs_i, ctgs_i, goal_i, timesteps_i, attention_mask_i, dt_i, time_sec_i, ix

def ros_model_inference_dyn(model, data_stats, state_init, state_final, rtg_perc=1., ctg_perc=1., rtg=None, ctg_clipped=True):
    # Get dimensions and statistics from the dataset
    n_state = data_stats['states_mean'].shape[1]
    n_time = data_stats['states_mean'].shape[0]
    n_action = data_stats['actions_mean'].shape[1]
    data_stats = copy.deepcopy(data_stats)
    data_stats['states_mean'] = data_stats['states_mean'].float().to(device)
    data_stats['states_std'] = data_stats['states_std'].float().to(device)
    data_stats['actions_mean'] = data_stats['actions_mean'].float().to(device)
    data_stats['actions_std'] = data_stats['actions_std'].float().to(device)
    data_stats['goal_mean'] = data_stats['goal_mean'].float().to(device)
    data_stats['goal_std'] = data_stats['goal_std'].float().to(device)

    # Unnormalize the data sample and compute orbital period (data sample is composed by tensors on the cpu)
    states_i, actions_i, rtgs_i, ctgs_i, goal_i, timesteps_i, attention_mask_i, dt, time_sec, ix = state_init_final2sample(data_stats, state_init, state_final)
    states_i = states_i.to(device)
    rtgs_i = rtgs_i.to(device)
    goal_i = goal_i.to(device)
    timesteps_i = timesteps_i.long().to(device)
    attention_mask_i = attention_mask_i.long().to(device)
    dt = dt.item()
    time_sec = np.array(time_sec[0])
    obs_pos, obs_rad = torch.tensor(np.copy(obs['position'])).to(device), torch.tensor(np.copy(obs['radius'])).to(device)
    obs_rad = (obs_rad + robot_radius)*safety_margin
    ff_model = FreeflyerModel()
    Ak, B_imp = torch.tensor(ff_model.Ak).to(device).float(), torch.tensor(ff_model.B_imp).to(device).float()

    # Retrieve decoded states and actions for different inference cases
    xypsi_dyn = torch.empty(size=(n_state, n_time), device=device).float()
    dv_dyn = torch.empty(size=(n_action, n_time), device=device).float()
    states_dyn = torch.empty(size=(1, n_time, n_state), device=device).float()
    actions_dyn = torch.zeros(size=(1, n_time, n_action), device=device).float()
    rtgs_dyn = torch.empty(size=(1, n_time, 1), device=device).float()
    ctgs_dyn = torch.empty(size=(1, n_time, 1), device=device).float()

    runtime0_DT = time.time()
    # Dynamics-in-the-loop initialization
    states_dyn[:,0,:] = states_i[:,0,:]
    if rtg is None:
        rtgs_dyn[:,0,:] = rtgs_i[:,0,:]*rtg_perc
    else:
        rtgs_dyn[:,0,:] = rtg
    ctgs_dyn[:,0,:] = ctgs_i[:,0,:]*ctg_perc
    xypsi_dyn[:, 0] = (states_dyn[:,0,:] * data_stats['states_std'][0]) + data_stats['states_mean'][0]
    
    # For loop trajectory generation
    for t in np.arange(n_time):
        
        ##### Dynamics inference        
        # Compute action pred for dynamics model
        with torch.no_grad():
            output_dyn = model(
                states=states_dyn[:,:t+1,:],
                actions=actions_dyn[:,:t+1,:],
                goal=goal_i[:,:t+1,:],
                returns_to_go=rtgs_dyn[:,:t+1,:],
                constraints_to_go=ctgs_dyn[:,:t+1,:],
                timesteps=timesteps_i[:,:t+1],
                attention_mask=attention_mask_i[:,:t+1],
                return_dict=False,
            )
            (_, action_preds_dyn) = output_dyn

        action_dyn_t = action_preds_dyn[0,t]
        actions_dyn[:,t,:] = action_dyn_t
        dv_dyn[:, t] = (action_dyn_t * (data_stats['actions_std'][t]+1e-6)) + data_stats['actions_mean'][t]

        # Dynamics propagation of state variable 
        if t != n_time-1:
            xypsi_dyn[:, t+1] = Ak @ (xypsi_dyn[:, t] + B_imp @ dv_dyn[:, t])
            states_dyn[:,t+1,:] = (xypsi_dyn[:,t+1] - data_stats['states_mean'][t+1]) / (data_stats['states_std'][t+1] + 1e-6)
            
            reward_dyn_t = - torch.linalg.norm(dv_dyn[:, t], ord=1)
            rtgs_dyn[:,t+1,:] = rtgs_dyn[0,t] - reward_dyn_t
            viol_dyn = torch_check_koz_constraint(xypsi_dyn[:,t+1], obs_pos, obs_rad)
            ctgs_dyn[:,t+1,:] = ctgs_dyn[0,t] - (viol_dyn if (not ctg_clipped) else 0)
            actions_dyn[:,t+1,:] = 0

    # Pack trajectory's data in a dictionary and compute runtime
    runtime1_DT = time.time()
    runtime_DT = runtime1_DT - runtime0_DT
    DT_trajectory = {
        'xypsi_dyn' : xypsi_dyn.cpu().numpy(),
        'dv_dyn' : dv_dyn.cpu().numpy(),
        'time' : time_sec
    }

    return DT_trajectory, runtime_DT

def ros_model_inference_dyn_time(model, data_stats, state_init, state_final, rtg_perc=1., ctg_perc=1., rtg=None, ttg=None, ctg_clipped=True, chunksize=None, end_on_ttg=False):
    # Get dimensions and statistics from the dataset
    n_state = data_stats['states_mean'].shape[1]
    n_action = data_stats['actions_mean'].shape[1]
    data_stats = copy.deepcopy(data_stats)
    data_stats['states_mean'] = data_stats['states_mean'].float().to(device)
    data_stats['states_std'] = data_stats['states_std'].float().to(device)
    data_stats['actions_mean'] = data_stats['actions_mean'].float().to(device)
    data_stats['actions_std'] = data_stats['actions_std'].float().to(device)
    data_stats['goal_mean'] = data_stats['goal_mean'].float().to(device)
    data_stats['goal_std'] = data_stats['goal_std'].float().to(device)
    data_stats['ttgs_mean'] = data_stats['ttgs_mean'].float().to(device)
    data_stats['ttgs_std'] = data_stats['ttgs_std'].float().to(device)

    # Unnormalize the data sample and compute orbital period (data sample is composed by tensors on the cpu)
    final_time = ttg if (not ttg is None) else T_nominal
    states_i, actions_i, rtgs_i, ctgs_i, ttgs_i, goal_i, timesteps_i, attention_mask_i, dt, time_sec, ix = state_init_final2sample(data_stats, state_init, state_final, final_time)
    states_i = states_i.to(device)
    rtgs_i = rtgs_i.to(device)
    ctgs_i = ctgs_i.to(device)
    ttgs_i = ttgs_i.to(device)
    goal_i = goal_i.to(device)
    timesteps_i = timesteps_i.long().to(device)
    attention_mask_i = attention_mask_i.long().to(device)
    dt = dt.item()
    time_sec = np.array(time_sec[0])
    obs_pos, obs_rad = torch.tensor(np.copy(obs['position'])).to(device), torch.tensor(np.copy(obs['radius'])).to(device)
    obs_rad = (obs_rad + robot_radius)*safety_margin
    ff_model = FreeflyerModel()
    Ak, B_imp = torch.tensor(ff_model.Ak).to(device).float(), torch.tensor(ff_model.B_imp).to(device).float()
    
    # Time characteristics
    if end_on_ttg:
        if not(ttg is None):
            n_time = round(ttg/dt)
        else:
            raise ValueError('Requested to end on ttg, but ttg has not been provided!')
    else:
        n_time = n_time_max
    if chunksize is None:
        chunksize = n_time

    # Retrieve decoded states and actions for different inference cases
    xypsi_dyn = torch.empty(size=(n_state, n_time), device=device).float()
    dv_dyn = torch.empty(size=(n_action, n_time), device=device).float()
    states_dyn = torch.empty(size=(1, n_time, n_state), device=device).float()
    actions_dyn = torch.zeros(size=(1, n_time, n_action), device=device).float()
    rtgs_dyn = torch.empty(size=(1, n_time, 1), device=device).float()
    ctgs_dyn = torch.empty(size=(1, n_time, 1), device=device).float()
    ttgs_pred_dyn = torch.zeros(size=(1, n_time), device=device).float()
    ttgs_dyn = torch.zeros(size=(1, n_time, 1), device=device).float()

    runtime0_DT = time.time()
    # Dynamics-in-the-loop initialization
    states_dyn[:,0,:] = states_i[:,0,:]
    if rtg is None:
        rtgs_dyn[:,0,:] = rtgs_i[:,0,:]*rtg_perc
    else:
        rtgs_dyn[:,0,:] = rtg
    ctgs_dyn[:,0,:] = ctgs_i[:,0,:]*ctg_perc
    if not (ttg is None):
        ttgs_pred_dyn[:,0] = ttg
        ttgs_dyn[:,0,:] = (ttg - data_stats['ttgs_mean'][0])/(data_stats['ttgs_std'][0]+1e-6)
    xypsi_dyn[:, 0] = (states_dyn[:,0,:] * data_stats['states_std'][0]) + data_stats['states_mean'][0]
    
    # For loop trajectory generation
    for t in np.arange(n_time):
        
        # Time interval to consider
        start_t = max(t - chunksize + 1, 0)
        end_t = t + 1

        ##### Dynamics inference
        # Compute time to go pred for dynamics model
        if (ttg is None) and ('predict_time' in model.__dict__['_modules'].keys()):
            with torch.no_grad():
                output_dyn = model(
                    states=states_dyn[:,start_t:end_t],
                    actions=actions_dyn[:,start_t:end_t],
                    goal=goal_i[:,start_t:end_t],
                    returns_to_go=rtgs_dyn[:,start_t:end_t],
                    constraints_to_go=ctgs_dyn[:,start_t:end_t],
                    times_to_go=ttgs_dyn[:,start_t:end_t],
                    timesteps=timesteps_i[:,:np.minimum(end_t,chunksize)],
                    attention_mask=attention_mask_i[:,start_t:end_t],
                    return_dict=False
                )
            ttg_preds_dyn = output_dyn[2]
            ttg_dyn_t = ttg_preds_dyn[0,-1] ######## camabiare indice per ttg_preds_ol e analoghi
            ttgs_dyn[:,t,:] = ttg_preds_dyn[0,-1]
            ttgs_pred_dyn[:,t] = (ttg_dyn_t * (data_stats['ttgs_std'][t]+1e-6)) + data_stats['ttgs_mean'][t]
        
        # Compute action pred for dynamics model
        with torch.no_grad():
            output_dyn = model(
                states=states_dyn[:,start_t:end_t],
                actions=actions_dyn[:,start_t:end_t],
                goal=goal_i[:,start_t:end_t],
                returns_to_go=rtgs_dyn[:,start_t:end_t],
                constraints_to_go=ctgs_dyn[:,start_t:end_t],
                times_to_go=ttgs_dyn[:,start_t:end_t],
                timesteps=timesteps_i[:,:np.minimum(end_t,chunksize)],
                attention_mask=attention_mask_i[:,start_t:end_t],
                return_dict=False,
            )
        action_preds_dyn = output_dyn[1]
        action_dyn_t = action_preds_dyn[0,-1]
        actions_dyn[:,t,:] = action_dyn_t
        dv_dyn[:, t] = (action_dyn_t * (data_stats['actions_std'][t]+1e-6)) + data_stats['actions_mean'][t]

        # Dynamics propagation of state variable 
        if t != n_time-1:
            xypsi_dyn[:, t+1] = Ak @ (xypsi_dyn[:, t] + B_imp @ dv_dyn[:, t])
            states_dyn[:,t+1,:] = (xypsi_dyn[:,t+1] - data_stats['states_mean'][t+1]) / (data_stats['states_std'][t+1] + 1e-6)
            
            # Update reward, constraints and time to go
            reward_dyn_t = - torch.linalg.norm(dv_dyn[:, t], ord=1)
            rtgs_dyn[:,t+1,:] = rtgs_dyn[0,t] - reward_dyn_t
            viol_dyn = torch_check_koz_constraint(xypsi_dyn[:,t+1], obs_pos, obs_rad)
            ctgs_dyn[:,t+1,:] = ctgs_dyn[0,t] - (viol_dyn if (not ctg_clipped) else 0)
            if not (ttg is None):
                ttgs_pred_dyn[:,t+1] = np.maximum(ttgs_pred_dyn[0, t].item() - dt, 0)
                ttgs_dyn[:,t+1,:] = (ttgs_pred_dyn[0,t+1] - data_stats['ttgs_mean'][t+1])/(data_stats['ttgs_std'][t+1]+1e-6)
            else:
                ttgs_dyn[:,t+1,:] = 0
            actions_dyn[:,t+1,:] = 0

    # Pack trajectory's data in a dictionary and compute runtime
    runtime1_DT = time.time()
    runtime_DT = runtime1_DT - runtime0_DT
    DT_trajectory = {
        'xypsi_dyn' : xypsi_dyn.cpu().numpy(),
        'dv_dyn' : dv_dyn.cpu().numpy(),
        'ttgs_dyn' : ttgs_pred_dyn.cpu().numpy(),
        'time' : time_sec
    }

    return DT_trajectory, runtime_DT

def torch_check_koz_constraint(states, obs_positions, obs_radii):

    constr_koz = torch.norm(states[None,:2] - obs_positions, 2, dim=1) - obs_radii
    constr_koz_violation = (1*(constr_koz <= 0)).sum().item()

    return constr_koz_violation
