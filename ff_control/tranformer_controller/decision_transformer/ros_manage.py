import os
import sys
import argparse

root_folder = os.path.abspath(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.append(root_folder)

import numpy as np
import copy

import torch

from transformers import DecisionTransformerConfig
from accelerate import Accelerator

from decision_transformer.art import AutonomousFreeflyerTransformer
from dynamics.freeflyer import FreeflyerModel
from optimization.ff_scenario import obs, safety_margin, robot_radius, dt, T
import time
# select device based on availability of GPU
verbose = False # set to True to get additional print statements
device = torch.device('cuda') if torch.cuda.is_available() else torch.device('cpu')
print(device)

'''
    TODO: 
        
'''

def get_only_DT_model(model_name, n_state, n_action):
    # DT model creation
    config = DecisionTransformerConfig(
        state_dim=n_state, 
        act_dim=n_action,
        hidden_size=384,
        max_ep_len=100,
        vocab_size=1,
        action_tanh=False,
        n_positions=1024,
        n_layer=6,
        n_head=6,
        n_inner=None,
        resid_pdrop=0.1,
        embd_pdrop=0.1,
        attn_pdrop=0.1,
        )
    model = AutonomousFreeflyerTransformer(config)
    model_size = sum(t.numel() for t in model.parameters())
    print(f"GPT size: {model_size/1000**2:.1f}M parameters")
    model.to(device);

    # DT optimizer and accelerator
    accelerator = Accelerator(mixed_precision='no', gradient_accumulation_steps=8)
    model = accelerator.prepare(model)
    accelerator.load_state(root_folder + '/decision_transformer/saved_files/checkpoints/' + model_name)

    return model.eval()

def state_init_final2sample(data_stats, state_init, state_final):
    n_time = data_stats['states_mean'].shhape[0]
    states_i = ((torch.tensor(np.repeat(state_init[None,:], 100, axis=0)) - data_stats['states_mean'])/(data_stats['states_std'] + 1e-6))[None,:,:]
    actions_i = torch.zeros((100,3))[None,:,:]
    rtgs_i = torch.zeros((100,))[None,:,None]
    ctgs_i = torch.zeros((100,))[None,:,None]
    goal_i = ((torch.tensor(np.repeat(state_final[None,:], 100, axis=0)) - data_stats['goal_mean'])/(data_stats['goal_std'] + 1e-6))[None,:,:]
    timesteps_i = (torch.tensor([[i for i in range(n_time)] for _ in ix]).view(n_time).long())[None,:]
    attention_mask_i = (torch.ones(1, n_time).view(n_time).long())[None,:]
    dt = torch.tensor([dt]).float()[None,:]
    time_sec = torch.arange(0,T+dt/2,dt).float()[None,:]
    ix = torch.tensor([0])
    return states_i, actions_i, rtgs_i, ctgs_i, goal_i, timesteps_i, attention_mask_i, dt, time_sec, ix

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

def torch_check_koz_constraint(states, obs_positions, obs_radii):

    constr_koz = torch.norm(states[None,:2] - obs_positions, 2, dim=1) - obs_radii
    constr_koz_violation = (1*(constr_koz <= 0)).sum().item()

    return constr_koz_violation
