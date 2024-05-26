import os
import sys

root_folder = os.path.abspath(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.append(root_folder)

import numpy as np
import numpy.linalg as la
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Rectangle
import copy

import torch
from torch.utils.data import Dataset
from torch.utils.data.dataloader import DataLoader
from torch.optim import AdamW

from transformers import DecisionTransformerConfig, DecisionTransformerModel
from accelerate import Accelerator

from decision_transformer.art import AutonomousFreeflyerTransformer_pred_time
from dynamics.freeflyer_time import FreeflyerModel, check_koz_constraint
from optimization.ff_scenario_time import obs, safety_margin, robot_radius, table, n_time_max
import time
# select device based on availability of GPU
verbose = False # set to True to get additional print statements
device = torch.device('cuda') if torch.cuda.is_available() else torch.device('cpu')
print(device)

'''
    TODO: 
        - normalize:
            1) currently in the normalization without timestep dependency the dimension of "states_mean", "states_std",.... is (100xsize_data) just for portability with precedent version of "torch_inference...." --> TO BE REMOVED IN NEXT VERSIONS
'''

class RpodDataset(Dataset):
    # Create a Dataset object
    def __init__(self, data, mdp_constr, chunksize=None, target=False):
        self.data_stats = data['data_stats']
        self.data = data
        self.n_data, self.max_len, self.n_state = self.data['states'].shape
        self.n_action = self.data['actions'].shape[2]
        self.mdp_constr = mdp_constr
        self.target = target
        self.chunksize = chunksize if not(chunksize is None) else self.max_len

    def __len__(self):
        return len(self.data)

    def __getitem__(self, idx):
        ix = torch.randint(self.n_data, (1,))
        # Select the chunk
        time_discr = self.data['data_param']['time_discr'][ix].item()
        final_time = self.data['data_param']['final_time'][ix].item()
        time_index_final = round(final_time/time_discr)
        if time_index_final > self.chunksize:
            if np.random.choice([0,1],1) == 0:
                time_indexes = np.arange(0, self.chunksize)
            else:
                time_indexes = np.arange(time_index_final - self.chunksize, time_index_final)
        else:
            time_indexes = np.arange(0, self.chunksize)
        states = torch.stack([self.data['states'][i, time_indexes, :]
                        for i in ix]).view(self.chunksize, self.n_state).float()
        actions = torch.stack([self.data['actions'][i, time_indexes, :]
                        for i in ix]).view(self.chunksize, self.n_action).float()
        rtgs = torch.stack([self.data['rtgs'][i, time_indexes]
                        for i in ix]).view(self.chunksize, 1).float()
        goal = torch.stack([self.data['goal'][i, time_indexes, :]
                        for i in ix]).view(self.chunksize, self.n_state).float()
        timesteps = torch.tensor([[i for i in range(self.chunksize)] for _ in ix]).view(self.chunksize).long()
        attention_mask = torch.ones(1, self.chunksize).view(self.chunksize).long()

        time_sec = self.data['data_param']['time_sec'][ix, time_indexes].reshape((1, self.chunksize))

        if self.target == False:
            if not self.mdp_constr:
                return states, actions, rtgs, goal, timesteps, attention_mask, time_discr, time_sec, ix
            else:
                ctgs = torch.stack([self.data['ctgs'][i, time_indexes]
                            for i in ix]).view(self.chunksize, 1).float()
                ttgs = torch.stack([self.data['ttgs'][i, time_indexes]
                            for i in ix]).view(self.chunksize, 1).float()
                return states, actions, rtgs, ctgs, ttgs, goal, timesteps, attention_mask, time_discr, time_sec, ix
        else:
            target_states = torch.stack([self.data['target_states'][i, time_indexes, :]
                            for i in ix]).view(self.chunksize-1, self.n_state).float()
            target_actions = torch.stack([self.data['target_actions'][i, time_indexes, :]
                            for i in ix]).view(self.chunksize, self.n_action).float()
            
            if not self.mdp_constr:
                return states, actions, rtgs, goal, target_states, target_actions, timesteps, attention_mask, time_discr, time_sec, ix
            else:
                ctgs = torch.stack([self.data['ctgs'][i, time_indexes]
                            for i in ix]).view(self.chunksize, 1).float()
                ttgs = torch.stack([self.data['ttgs'][i, time_indexes]
                            for i in ix]).view(self.chunksize, 1).float()
                return states, actions, rtgs, ctgs, ttgs, goal, target_states, target_actions, timesteps, attention_mask, time_discr, time_sec, ix
    
    def getix(self, ix):
        ix = [ix]
        # Select the chunk
        time_discr = self.data['data_param']['time_discr'][ix].item()
        final_time = self.data['data_param']['final_time'][ix].item()
        time_index_final = round(final_time/time_discr)
        if time_index_final > self.chunksize:
            if np.random.choice([0,1],1) == 0:
                time_indexes = np.arange(0, self.chunksize)
            else:
                time_indexes = np.arange(time_index_final - self.chunksize, time_index_final)
        else:
            time_indexes = np.arange(0, self.chunksize)
        states = torch.stack([self.data['states'][i, time_indexes, :]
                        for i in ix]).view(self.chunksize, self.n_state).float().unsqueeze(0)
        actions = torch.stack([self.data['actions'][i, time_indexes, :]
                        for i in ix]).view(self.chunksize, self.n_action).float().unsqueeze(0)
        rtgs = torch.stack([self.data['rtgs'][i, time_indexes]
                        for i in ix]).view(self.chunksize, 1).float().unsqueeze(0)
        goal = torch.stack([self.data['goal'][i, time_indexes, :]
                        for i in ix]).view(self.chunksize, self.n_state).float().unsqueeze(0)
        timesteps = torch.tensor([[i for i in range(self.chunksize)] for _ in ix]).view(self.chunksize).long().unsqueeze(0)
        attention_mask = torch.ones(1, self.chunksize).view(self.chunksize).long().unsqueeze(0)

        time_discr = torch.tensor(time_discr)
        time_sec = torch.tensor(self.data['data_param']['time_sec'][ix].reshape((1, self.chunksize))).unsqueeze(0)

        if self.target == False:
            if not self.mdp_constr:
                return states, actions, rtgs, goal, timesteps, attention_mask, time_discr, time_sec, ix
            else:
                ctgs = torch.stack([self.data['ctgs'][i, time_indexes]
                            for i in ix]).view(self.chunksize, 1).float()
                ttgs = torch.stack([self.data['ttgs'][i, time_indexes]
                            for i in ix]).view(self.chunksize, 1).float()
                return states, actions, rtgs, ctgs, ttgs, goal, timesteps, attention_mask, time_discr, time_sec, ix
        else:
            target_states = torch.stack([self.data['target_states'][i, time_indexes, :]
                            for i in ix]).view(self.chunksize-1, self.n_state).float().unsqueeze(0)
            target_actions = torch.stack([self.data['target_actions'][i, time_indexes, :]
                            for i in ix]).view(self.chunksize, self.n_action).float().unsqueeze(0)

            if not self.mdp_constr:
                return states, actions, rtgs, goal, target_states, target_actions, timesteps, attention_mask, time_discr, time_sec, ix
            else:
                ctgs = torch.stack([self.data['ctgs'][i, time_indexes]
                            for i in ix]).view(self.chunksize, 1).float()
                ttgs = torch.stack([self.data['ttgs'][i, time_indexes]
                            for i in ix]).view(self.chunksize, 1).float()
                return states, actions, rtgs, ctgs, ttgs, goal, target_states, target_actions, timesteps, attention_mask, time_discr, time_sec, ix

    def get_data_size(self):
        return self.n_data

def transformer_import_config(model_name):
    config = {}
    config['model_name'] = model_name
    config['mdp_constr'] = True
    config['timestep_norm'] = False
    if 'time_whole_table' in model_name:
        from optimization.ff_scenario_time import dataset_scenario, chunksize
        config['chunksize'] = chunksize
        if not('time_whole_table' in dataset_scenario):
            raise NameError('Transformer model for scenario with the varying final time and considering the whole table as start and goal region, but "dataset_scenario" in ff_scenario_time.py is \"' + dataset_scenario + '\"')
        else:
            config['dataset_scenario'] = dataset_scenario
    elif 'time' in model_name:
        from optimization.ff_scenario_time import dataset_scenario, chunksize
        config['chunksize'] = chunksize
        if not('time' in dataset_scenario):
            raise NameError('Transformer model for scenario with varying time, but "dataset_scenario" in ff_scenario_time.py is \"' + dataset_scenario + '\"')
        else:
            config['dataset_scenario'] = dataset_scenario
    else:
        raise NameError('No transformer model with name', model_name, 'found!')
    
    if ('chunk' in model_name and (not('None' in model_name))) and (config['chunksize'] is None):
        raise NameError('Transformer model for scenario with time chunks, but "chunksize" in ff_scenario_time.py is \"' + str(config['chunksize']) + '\"')
    elif ((not ('chunk' in model_name)) or ('chunkNone' in model_name)) and (not(config['chunksize'] is None)):
        raise NameError('Transformer model for scenario without time chunks, but "chunksize" in ff_scenario_time.py is \"' + str(config['chunksize']) + '\"')
    
    return config

def get_train_val_test_data(mdp_constr, dataset_scenario, timestep_norm, chunksize):

    # Import and normalize torch dataset, then save data statistics
    torch_data, data_param = import_dataset_for_DT_eval_vXX(dataset_scenario, mdp_constr)
    states_norm, states_mean, states_std = normalize(torch_data['torch_states'], timestep_norm)
    actions_norm, actions_mean, actions_std = normalize(torch_data['torch_actions'], timestep_norm)
    goal_norm, goal_mean, goal_std = normalize(torch_data['torch_goal'], timestep_norm)
    target_states_norm = states_norm[:,1:,:].clone().detach()
    target_actions_norm = actions_norm.clone().detach()
    if mdp_constr:
        rtgs_norm, rtgs_mean, rtgs_std = torch_data['torch_rtgs'], None, None
        ctgs_norm, ctgs_mean, ctgs_std = torch_data['torch_ctgs'], None, None
        ttgs_norm, ttgs_mean, ttgs_std = normalize(torch_data['torch_ttgs'], timestep_norm)
    else:
        rtgs_norm, rtgs_mean, rtgs_std = normalize(torch_data['torch_rtgs'], timestep_norm)
    
    data_stats = {
        'states_mean' : states_mean,
        'states_std' : states_std,
        'actions_mean' : actions_mean,
        'actions_std' : actions_std,
        'rtgs_mean' : rtgs_mean,
        'rtgs_std' : rtgs_std,
        'ctgs_mean' : ctgs_mean if mdp_constr else None,
        'ctgs_std' : ctgs_std if mdp_constr else None,
        'ttgs_mean' : ttgs_mean if mdp_constr else None,
        'ttgs_std' : ttgs_std if mdp_constr else None,
        'goal_mean' : goal_mean,
        'goal_std' : goal_std
    }

    # Split dataset into training and validation
    n = int(0.9*states_norm.shape[0])
    train_data = {
        'states' : states_norm[:n, :],
        'actions' : actions_norm[:n, :],
        'rtgs' : rtgs_norm[:n, :],
        'ctgs' : ctgs_norm[:n, :] if mdp_constr else None,
        'ttgs' : ttgs_norm[:n, :] if mdp_constr else None, 
        'target_states' : target_states_norm[:n, :],
        'target_actions' : target_actions_norm[:n, :],
        'goal' : goal_norm[:n, :],
        'data_param' : {
            'time_discr' : data_param['time_discr'][:n],
            'time_sec' : data_param['time_sec'][:n, :],
            'final_time' : data_param['final_time'][:n]
            },
        'data_stats' : data_stats
        }
    val_data = {
        'states' : states_norm[n:, :],
        'actions' : actions_norm[n:, :],
        'rtgs' : rtgs_norm[n:, :],
        'ctgs' : ctgs_norm[n:, :] if mdp_constr else None,
        'ttgs' : ttgs_norm[n:, :] if mdp_constr else None,
        'target_states' : target_states_norm[n:, :],
        'target_actions' : target_actions_norm[n:, :],
        'goal' : goal_norm[n:, :],
        'data_param' : {
            'time_discr' : data_param['time_discr'][n:],
            'time_sec' : data_param['time_sec'][n:, :],
            'final_time' : data_param['final_time'][:n]
            },
        'data_stats' : data_stats
        }
    
    # Create datasets
    train_dataset = RpodDataset(train_data, mdp_constr, chunksize)
    val_dataset = RpodDataset(val_data, mdp_constr, chunksize)
    test_dataset = RpodDataset(val_data, mdp_constr)
    datasets = (train_dataset, val_dataset, test_dataset)

    # Create data loaders
    train_loader = DataLoader(
        train_dataset,
        sampler=torch.utils.data.RandomSampler(
            train_dataset, replacement=True, num_samples=int(1e10)),
        shuffle=False,
        pin_memory=True,
        batch_size=4,
        num_workers=0,
    )
    eval_loader = DataLoader(
        val_dataset,
        sampler=torch.utils.data.RandomSampler(
            val_dataset, replacement=True, num_samples=int(1e10)),
        shuffle=False,
        pin_memory=True,
        batch_size=4,
        num_workers=0,
    )
    test_loader = DataLoader(
        test_dataset,
        sampler=torch.utils.data.RandomSampler(
            test_dataset, replacement=True, num_samples=int(1e10)),
        shuffle=False,
        pin_memory=True,
        batch_size=1,
        num_workers=0,
    )
    dataloaders = (train_loader, eval_loader, test_loader)
    
    return datasets, dataloaders

def import_dataset_for_DT_eval_vXX(dataset_scenario, mdp_constr):
    # Load the data
    print('Loading data from root/dataset/torch/...', end='')

    data_dir = root_folder + '/dataset/' + dataset_scenario
    data_dir_torch = root_folder + '/dataset/' + dataset_scenario + '/torch/v05'
    states_cvx = torch.load(data_dir_torch + '/torch_states_cvx.pth')
    states_scp = torch.load(data_dir_torch + '/torch_states_scp.pth')
    actions_cvx = torch.load(data_dir_torch + '/torch_actions_cvx.pth')
    actions_scp = torch.load(data_dir_torch + '/torch_actions_scp.pth')
    rtgs_cvx = torch.load(data_dir_torch + '/torch_rtgs_cvx.pth')[:,:,None]
    rtgs_scp = torch.load(data_dir_torch + '/torch_rtgs_scp.pth')[:,:,None]
    ctgs_cvx = torch.load(data_dir_torch + '/torch_ctgs_cvx.pth')[:,:,None]
    ctgs_scp = torch.load(data_dir_torch + '/torch_ctgs_scp.pth')[:,:,None]
    data_param = np.load(data_dir + '/dataset-ff-v05-param.npz', allow_pickle=True)

    # Extend time sequence
    n_time_max = states_cvx.shape[1]
    ttgs = -100*np.ones(rtgs_cvx.shape)
    time_sec = -100*np.ones(states_cvx.shape[:2])
    for n_data, (time_i, final_time_i) in enumerate(zip(data_param['time'], data_param['final_time'])):
        dt_i = time_i[1] - time_i[0]
        time_sec[n_data, :] = np.hstack((time_i, time_i[-1] + dt_i*np.arange(1, n_time_max - time_i.shape[0] +1)))
        ttgs_i = np.arange(final_time_i, 0, -dt_i)
        ttgs[n_data, :, 0] = np.concatenate((ttgs_i, np.zeros((n_time_max - time_i.shape[0]))))
    ttgs = torch.from_numpy(ttgs)

    print('Almost done, DATA IS NOT SHUFFLED YET.\n')

    # Output dictionary
    if mdp_constr:
        perm = np.load(data_dir_torch + '/permutation.npy')
        torch_states = torch.concatenate((states_scp, states_cvx), axis=0)[perm]
        torch_actions = torch.concatenate((actions_scp, actions_cvx), axis=0)[perm]
        torch_rtgs = torch.concatenate((rtgs_scp, rtgs_cvx), axis=0)[perm]
        torch_ctgs = torch.concatenate((ctgs_scp, ctgs_cvx), axis=0)[perm]
        torch_ttgs = torch.concatenate((ttgs, ttgs), axis=0)[perm]
        goal_timeseq = torch.tensor(np.repeat(data_param['target_state'][:,None,:], torch_states.shape[1], axis=1))
        torch_goal = torch.concatenate((goal_timeseq, goal_timeseq), axis=0)[perm]
        data_param = {
            'time_discr' : np.concatenate((data_param['dtime'], data_param['dtime']), axis=0)[perm],
            'time_sec' : np.concatenate((time_sec, time_sec), axis=0)[perm],
            'final_time' : np.concatenate((data_param['final_time'], data_param['final_time']), axis=0)[perm]
        }
        print('Completed, DATA IS SHUFFLED.\n')
    else:
        torch_states = states_scp
        torch_actions = actions_scp
        torch_rtgs = rtgs_scp
        torch_ctgs = ctgs_scp
        torch_ttgs = ttgs
        torch_goal = torch.tensor(np.repeat(data_param['target_state'][:,None,:], torch_states.shape[1], axis=1))
        data_param = {
            'time_discr' : data_param['dtime'],
            'time_sec' : time_sec,
            'final_time' : data_param['final_time']
        }

    torch_data = {
        'torch_states' : torch_states,
        'torch_actions' : torch_actions,
        'torch_rtgs' : torch_rtgs,
        'torch_ctgs' : torch_ctgs,
        'torch_ttgs' : torch_ttgs,
        'torch_goal' : torch_goal
    }

    return torch_data, data_param

def normalize(data, timestep_norm):
    # Normalize and return normalized data, mean and std
    if timestep_norm:
        data_mean = data.mean(dim=0)
        data_std = data.std(dim=0)
        data_norm = (data - data_mean)/(data_std + 1e-6)
    else:
        time_length, size_data = data.shape[1:]
        data_mean = torch.ones((time_length, size_data)) * data.view(-1,size_data).mean(dim=0)
        data_std = torch.ones((time_length, size_data)) * data.view(-1,size_data).std(dim=0)
        data_norm = (data - data_mean)/(data_std + 1e-6)

    return data_norm, data_mean, data_std

def get_DT_model(model_name, train_loader, eval_loader):
    # DT model creation
    config = DecisionTransformerConfig(
        state_dim=train_loader.dataset.n_state, 
        act_dim=train_loader.dataset.n_action,
        hidden_size=384,
        max_ep_len=n_time_max,
        vocab_size=1,
        action_tanh=False,
        n_positions=2048,
        n_layer=6,
        n_head=6,
        n_inner=None,
        resid_pdrop=0.1,
        embd_pdrop=0.1,
        attn_pdrop=0.1,
        )
    if 'ctgrtg' in model_name:
        model = AutonomousFreeflyerTransformer_pred_time(config)
    else:
        model = DecisionTransformerModel(config)
    model_size = sum(t.numel() for t in model.parameters())
    print(f"GPT size: {model_size/1000**2:.1f}M parameters")
    model.to(device);

    # DT optimizer and accelerator
    optimizer = AdamW(model.parameters(), lr=3e-5)
    accelerator = Accelerator(mixed_precision='no', gradient_accumulation_steps=8)
    model, optimizer, train_dataloader, eval_dataloader = accelerator.prepare(
        model, optimizer, train_loader, eval_loader
    )
    accelerator.load_state(root_folder + '/decision_transformer/saved_files/checkpoints/' + model_name)

    return model.eval()

def use_model_for_imitation_learning(model, test_loader, data_sample, rtg_perc=1., ctg_perc=1., rtg=None, ttg=None, use_dynamics=True, ctg_clipped=True, chunksize=None):
    # Get dimensions and statistics from the dataset
    n_state = test_loader.dataset.n_state
    n_time = test_loader.dataset.max_len
    n_action = test_loader.dataset.n_action
    data_stats = test_loader.dataset.data_stats
    if chunksize is None:
        chunksize = n_time

    # Unnormalize the data sample and compute orbital period
    if test_loader.dataset.mdp_constr:
        states_i, actions_i, rtgs_i, ctgs_i, ttgs_i, goal_i, timesteps_i, attention_mask_i, dt, time_sec, ix = data_sample
        ctgs_i = ctgs_i.view(1, n_time, 1)
        ttgs_i = ttgs_i.view(1, n_time, 1)
    else:
        states_i, actions_i, rtgs_i, goal_i, timesteps_i, attention_mask_i, dt, time_sec, ix = data_sample
    states_i_unnorm = (states_i * data_stats['states_std']) + data_stats['states_mean']
    actions_i_unnorm = (actions_i * data_stats['actions_std']) + data_stats['actions_mean']
    goal_i_unnorm = (goal_i * data_stats['goal_std']) + data_stats['goal_mean']
    if not test_loader.dataset.mdp_constr:
        rtgs_i_unnorm = (rtgs_i * data_stats['rtgs_std']) + data_stats['rtgs_mean']
    else:
        ttgs_i_unnorm = (ttgs_i * data_stats['ttgs_std']) + data_stats['ttgs_mean']
    dt = dt.item()
    time_sec = np.array(time_sec[0])
    obs_pos, obs_rad = np.copy(obs['position']), np.copy(obs['radius'])
    obs_rad = (obs_rad + robot_radius)*safety_margin

    # Retrieve decoded states and actions for different inference cases
    xypsi_true = np.empty(shape=(n_state, n_time), dtype=float)
    xypsi_ol = np.empty(shape=(n_state, n_time), dtype=float)
    xypsi_dyn = np.empty(shape=(n_state, n_time), dtype=float)
    dv_true = np.empty(shape=(n_action, n_time), dtype=float)
    dv_ol = np.empty(shape=(n_action, n_time), dtype=float)
    dv_dyn = np.empty(shape=(n_action, n_time), dtype=float)
    ttgs_true = np.empty(shape=(1, n_time), dtype=float)
    ttgs_pred_ol = np.empty(shape=(1, n_time), dtype=float)
    ttgs_pred_dyn = np.empty(shape=(1, n_time), dtype=float)

    # Open-loop initialization
    states_ol = states_i[:, 0, :][None, :, :].float().to(device)
    actions_ol = torch.zeros((1, 1, n_action), device=device).float()
    if rtg is None:
        rtgs_ol = rtgs_i[:, 0, :].view(1, 1, 1).float().to(device)*rtg_perc
    else:
        rtgs_ol = torch.tensor(rtg).view(1, 1, 1).float().to(device)
    #print(rtgs_ol)
    if test_loader.dataset.mdp_constr:
        ctgs_ol = ctgs_i[:, 0, :].view(1, 1, 1).float().to(device)*ctg_perc
        if ttg is None:
            ttgs_ol = torch.zeros((1, 1, 1), device=device).float()
        else:
            ttgs_ol = torch.tensor((ttg - data_stats['ttgs_mean'][0])/(data_stats['ttgs_std'][0]+1e-6)).view(1, 1, 1).float().to(device)
            ttgs_pred_ol[0,0] = ttg
    goal_ol = goal_i[:, 0, :][None, :, :].float().to(device)
    timesteps_ol = timesteps_i[:, 0][None, :].long().to(device)
    attention_mask_ol = attention_mask_i[:, 0][None, :].long().to(device)

    state_ol_t_unnorm = (states_ol[:,0,:].to(device) * data_stats['states_std'][0].to(device)) + data_stats['states_mean'][0].to(device)
    xypsi_ol[:, 0] = [state_ol_t_unnorm[0,i].item() for i in range(n_state)]

    # Dynamics-in-the-loop initialization
    if use_dynamics:
        states_dyn = states_i[:, 0, :][None, :, :].float().to(device)
        actions_dyn = torch.zeros((1, 1, n_action), device=device).float()
        if rtg is None:
            rtgs_dyn = rtgs_i[:, 0, :].view(1, 1, 1).float().to(device)*rtg_perc
        else:
            rtgs_dyn = torch.tensor(rtg).view(1, 1, 1).float().to(device)
        #print(rtgs_dyn)
        #rtgs_dyn = rtgs_i[:, 0, :].view(1, 1, 1).float().to(device)*rtg_perc
        if test_loader.dataset.mdp_constr:
            ctgs_dyn = ctgs_i[:, 0, :].view(1, 1, 1).float().to(device)*ctg_perc
            if ttg is None:
                ttgs_dyn = torch.zeros((1, 1, 1), device=device).float()
            else:
                ttgs_dyn = torch.tensor((ttg - data_stats['ttgs_mean'][0])/(data_stats['ttgs_std'][0]+1e-6)).view(1, 1, 1).float().to(device)
                ttgs_pred_dyn[0,0] = ttg
        goal_dyn = goal_i[:, 0, :][None, :, :].float().to(device)
        timesteps_dyn = timesteps_i[:, 0][None, :].long().to(device)
        attention_mask_dyn = attention_mask_i[:, 0][None, :].long().to(device)

        state_dyn_t_unnorm = (states_dyn[:,0,:].to(device) * data_stats['states_std'][0].to(device)) + data_stats['states_mean'][0].to(device)
        xypsi_dyn[:, 0] = [state_dyn_t_unnorm[0,i].item() for i in range(n_state)]
        ff_model = FreeflyerModel()

    # For loop trajectory generation
    for t in range(n_time):
        
        ##### Decode true data sample
        state_true_t = states_i_unnorm[0,t].cpu()
        action_true_t = actions_i_unnorm[0,t].cpu()
        ttg_true_t = ttgs_i_unnorm[0,t].cpu()
        xypsi_true[:, t] = [state_true_t[i].item() for i in range(n_state)]
        dv_true[:, t] = [action_true_t[i].item() for i in range(n_action)]
        ttgs_true[:, t] = ttg_true_t[0].item()

        # Time interval to consider
        start_t = max(t - chunksize + 1, 0)
        end_t = t + 1

        ##### Open-loop inference
        # Compute time pred for open-loop model
        if (ttg is None) and (test_loader.dataset.mdp_constr):
            with torch.no_grad():
                _, _, ttg_preds_ol = model(
                    states=states_ol[:,start_t:end_t].to(device),
                    actions=actions_ol[:,start_t:end_t].to(device),
                    goal=goal_ol[:,start_t:end_t].to(device),
                    returns_to_go=rtgs_ol[:,start_t:end_t].to(device),
                    constraints_to_go=ctgs_ol[:,start_t:end_t].to(device),
                    times_to_go=ttgs_ol[:,start_t:end_t].to(device),
                    timesteps=timesteps_ol[:,:np.minimum(end_t,chunksize)].to(device),
                    attention_mask=attention_mask_ol[:,start_t:end_t].to(device),
                    return_dict=False
                )
            ttg_ol_t = ttg_preds_ol[0,-1].cpu() ######## camabiare indice per ttg_preds_ol e analoghi
            ttgs_ol[:,-1,:] = ttg_preds_ol[0,-1][None,None,:].float()
            ttg_ol_t_unnorm = (ttg_ol_t.to(device) * (data_stats['ttgs_std'][t].to(device)+1e-6)) + data_stats['ttgs_mean'][t].to(device)
            ttgs_pred_ol[:, t] = ttg_ol_t_unnorm.item()

        # Compute action pred for open-loop model
        with torch.no_grad():
            _, action_preds_ol, _ = model(
                states=states_ol[:,start_t:end_t].to(device),
                actions=actions_ol[:,start_t:end_t].to(device),
                goal=goal_ol[:,start_t:end_t].to(device),
                returns_to_go=rtgs_ol[:,start_t:end_t].to(device),
                constraints_to_go=ctgs_ol[:,start_t:end_t].to(device),
                times_to_go=ttgs_ol[:,start_t:end_t].to(device),
                timesteps=timesteps_ol[:,:np.minimum(end_t,chunksize)].to(device),
                attention_mask=attention_mask_ol[:,start_t:end_t].to(device),
                return_dict=False
            )
        action_ol_t = action_preds_ol[0,-1].cpu() ######## camabiare indice per ttg_preds_ol e analoghi
        actions_ol[:,-1,:] = action_preds_ol[0,-1][None,None,:].float()
        action_ol_t_unnorm = (action_ol_t.to(device) * (data_stats['actions_std'][t].to(device)+1e-6)) + data_stats['actions_mean'][t].to(device)
        dv_ol[:, t] = [action_ol_t_unnorm[i].item() for i in range(n_action)]

        with torch.no_grad():
            state_preds_ol, _, _ = model(
                states=states_ol[:,start_t:end_t].to(device),
                actions=actions_ol[:,start_t:end_t].to(device),
                goal=goal_ol[:,start_t:end_t].to(device),
                returns_to_go=rtgs_ol[:,start_t:end_t].to(device),
                constraints_to_go=ctgs_ol[:,start_t:end_t].to(device),
                times_to_go=ttgs_ol[:,start_t:end_t].to(device),
                timesteps=timesteps_ol[:,:np.minimum(end_t,chunksize)].to(device),
                attention_mask=attention_mask_ol[:,start_t:end_t].to(device),
                return_dict=False
            )
        state_ol_t = state_preds_ol[0,-1].cpu()

        # Open-loop propagation of state variable
        if t != n_time-1:
            # State
            states_ol = torch.cat((states_ol, torch.tensor(state_preds_ol[:, -1][None,:,:]).to(device)), dim=1).float()
            state_ol_t_unnorm = (state_ol_t.to(device) * (data_stats['states_std'][t+1].to(device)+1e-6)) + data_stats['states_mean'][t+1].to(device)
            xypsi_ol[:, t+1] = [state_ol_t_unnorm[i].item() for i in range(n_state)]

            # Reward
            reward_ol_t = - la.norm(dv_ol[:, t], ord=1)
            rtgs_ol = torch.cat((rtgs_ol, rtgs_ol[0, -1].view(1, 1, 1) - reward_ol_t), dim=1)

            # Constraints violation
            _, viol_ol = check_koz_constraint(xypsi_ol[:,:t+2].T, obs_pos, obs_rad)
            if len(viol_ol.shape) == 2:
                viol_to_detract_ol = (viol_ol[:,-1].sum() if (not ctg_clipped) else 0)
            else:
                viol_to_detract_ol = (viol_ol[-1] if (not ctg_clipped) else 0)
            ctgs_ol = torch.cat((ctgs_ol, ctgs_ol[0, -1].view(1, 1, 1) - viol_to_detract_ol), dim=1)

            # Goal, action and time
            goal_ol = torch.cat((goal_ol, goal_i[:, t+1][None,:].to(device)), dim=1).float()
            timesteps_ol = torch.cat((timesteps_ol, timesteps_i[:, t+1][None,:].to(device)), dim=1).long()
            attention_mask_ol = torch.cat((attention_mask_ol, attention_mask_i[:, t+1][None,:].to(device)), dim=1).long()
            actions_ol = torch.cat((actions_ol, torch.zeros((1, 1, n_action), device=device).float()), dim=1)
            if ttg is None:
                ttgs_ol = torch.cat((ttgs_ol, torch.zeros((1, 1, 1), device=device).float()), dim=1)
            else:
                ttgs_pred_ol[:, t+1] = np.maximum(ttgs_pred_ol[0, t].item() - dt, 0)
                next_ttg_norm_ol = ((ttgs_pred_ol[0,t+1] - data_stats['ttgs_mean'][t+1].to(device))/(data_stats['ttgs_std'][t+1].to(device)+1e-6)).view(1,1,1).float()
                ttgs_ol = torch.cat((ttgs_ol, next_ttg_norm_ol), dim=1)
        
        ##### Dynamics inference        
        if use_dynamics:
            
            # Compute time pred for open-loop model
            if (ttg is None) and (test_loader.dataset.mdp_constr):
                with torch.no_grad():
                    _, _, ttg_preds_dyn = model(
                        states=states_dyn[:,start_t:end_t].to(device),
                        actions=actions_dyn[:,start_t:end_t].to(device),
                        goal=goal_dyn[:,start_t:end_t].to(device),
                        returns_to_go=rtgs_dyn[:,start_t:end_t].to(device),
                        constraints_to_go=ctgs_dyn[:,start_t:end_t].to(device),
                        times_to_go=ttgs_dyn[:,start_t:end_t].to(device),
                        timesteps=timesteps_dyn[:,:np.minimum(end_t,chunksize)].to(device),
                        attention_mask=attention_mask_dyn[:,start_t:end_t].to(device),
                        return_dict=False
                    )
                ttg_dyn_t = ttg_preds_dyn[0,-1].cpu()
                ttgs_dyn[:,-1,:] = ttg_preds_dyn[0,-1][None,None,:].float()
                ttg_dyn_t_unnorm = (ttg_dyn_t.to(device) * (data_stats['ttgs_std'][t].to(device)+1e-6)) + data_stats['ttgs_mean'][t].to(device)
                ttgs_pred_dyn[:, t] = ttg_dyn_t_unnorm.item()

            # Compute action pred for dynamics model
            with torch.no_grad():
                _, action_preds_dyn, _ = model(
                    states=states_dyn[:,start_t:end_t].to(device),
                    actions=actions_dyn[:,start_t:end_t].to(device),
                    goal=goal_dyn[:,start_t:end_t].to(device),
                    returns_to_go=rtgs_dyn[:,start_t:end_t].to(device),
                    constraints_to_go=ctgs_dyn[:,start_t:end_t].to(device),
                    times_to_go=ttgs_dyn[:,start_t:end_t].to(device),
                    timesteps=timesteps_dyn[:,:np.minimum(end_t,chunksize)].to(device),
                    attention_mask=attention_mask_dyn[:,start_t:end_t].to(device),
                    return_dict=False
                )
            action_dyn_t = action_preds_dyn[0,-1].cpu()
            actions_dyn[:,-1,:] = action_preds_dyn[0,-1][None,None,:].float()
            action_dyn_t_unnorm = (action_dyn_t.to(device) * (data_stats['actions_std'][t].to(device)+1e-6)) + data_stats['actions_mean'][t].to(device)
            dv_dyn[:, t] = [action_dyn_t_unnorm[i].item() for i in range(n_action)]

            # Dynamics propagation of state variable 
            if t != n_time-1:
                # State
                xypsi_dyn[:, t+1] = ff_model.Ak @ (xypsi_dyn[:, t] + ff_model.B_imp @ dv_dyn[:, t])
                states_dyn_norm = (torch.tensor(xypsi_dyn[:, t+1]).to(device) - data_stats['states_mean'][t+1].to(device)) / (data_stats['states_std'][t+1].to(device)+1e-6)
                states_dyn = torch.cat((states_dyn, states_dyn_norm[None,None,:]), dim=1).to(device).float()
                
                # Reward
                reward_dyn_t = - la.norm(dv_dyn[:, t], ord=1)
                rtgs_dyn = torch.cat((rtgs_dyn, rtgs_dyn[0, -1].view(1, 1, 1) - reward_dyn_t), dim=1)

                # Constraint violation
                _, viol_dyn = check_koz_constraint(xypsi_dyn[:,:t+2].T, obs_pos, obs_rad)
                if len(viol_dyn.shape) == 2:
                    viol_to_detract_dyn = (viol_dyn[:,-1].sum() if (not ctg_clipped) else 0)
                else:
                    viol_to_detract_dyn = (viol_dyn[-1] if (not ctg_clipped) else 0)
                ctgs_dyn = torch.cat((ctgs_dyn, ctgs_dyn[0, -1].view(1, 1, 1) - viol_to_detract_dyn), dim=1)

                # Goal, action and time
                goal_dyn = torch.cat((goal_dyn, goal_i[:, t+1][None,:].to(device)), dim=1).float()
                timesteps_dyn = torch.cat((timesteps_dyn, timesteps_i[:, t+1][None,:].to(device)), dim=1).long()
                attention_mask_dyn = torch.cat((attention_mask_dyn, attention_mask_i[:, t+1][None,:].to(device)), dim=1).long()
                actions_dyn = torch.cat((actions_dyn, torch.zeros((1, 1, n_action), device=device).float()), dim=1)
                if ttg is None:
                    ttgs_dyn = torch.cat((ttgs_dyn, torch.zeros((1, 1, 1), device=device).float()), dim=1)
                else:
                    ttgs_pred_dyn[:, t+1] = np.maximum(ttgs_pred_dyn[0, t].item() - dt, 0)
                    next_ttg_norm_dyn = ((ttgs_pred_dyn[0,t+1] - data_stats['ttgs_mean'][t+1].to(device))/(data_stats['ttgs_std'][t+1].to(device)+1e-6)).view(1,1,1).float()
                    ttgs_dyn = torch.cat((ttgs_dyn, next_ttg_norm_dyn), dim=1)

    # Pack trajectory's data in a dictionary
    DT_trajectory = {
        'xypsi_true' : xypsi_true,
        'xypsi_dyn' : xypsi_dyn,
        'xypsi_ol' : xypsi_ol,
        'dv_true' : dv_true,
        'dv_dyn' : dv_dyn,
        'dv_ol' : dv_ol,
        'ttgs_true' : ttgs_true,
        'ttgs_ol' : ttgs_pred_ol,
        'ttgs_dyn' : ttgs_pred_dyn,
        'time' : time_sec
    }

    return DT_trajectory

def torch_model_inference_dyn(model, test_loader, data_sample, rtg_perc=1., ctg_perc=1., rtg=None, ttg=None, ctg_clipped=True, chunksize=None, end_on_ttg=False):
    # Get dimensions and statistics from the dataset
    n_state = test_loader.dataset.n_state
    n_action = test_loader.dataset.n_action
    data_stats = copy.deepcopy(test_loader.dataset.data_stats)
    data_stats['states_mean'] = data_stats['states_mean'].float().to(device)
    data_stats['states_std'] = data_stats['states_std'].float().to(device)
    data_stats['actions_mean'] = data_stats['actions_mean'].float().to(device)
    data_stats['actions_std'] = data_stats['actions_std'].float().to(device)
    data_stats['goal_mean'] = data_stats['goal_mean'].float().to(device)
    data_stats['goal_std'] = data_stats['goal_std'].float().to(device)
    data_stats['ttgs_mean'] = data_stats['ttgs_mean'].float().to(device)
    data_stats['ttgs_std'] = data_stats['ttgs_std'].float().to(device)

    # Unnormalize the data sample and compute orbital period (data sample is composed by tensors on the cpu)
    if test_loader.dataset.mdp_constr:
        states_i, actions_i, rtgs_i, ctgs_i, ttgs_i, goal_i, timesteps_i, attention_mask_i, dt, time_sec, ix = data_sample
        ctgs_i = ctgs_i.view(1, -1, 1).to(device) # probably not needed??
        ttgs_i = ttgs_i.view(1, -1, 1).to(device)
    else:
        states_i, actions_i, rtgs_i, goal_i, timesteps_i, attention_mask_i, dt, time_sec, ix = data_sample
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
    
    # Time characteristics
    if end_on_ttg:
        if not(ttg is None):
            n_time = int(ttg/dt)
        else:
            raise ValueError('Requested to end on ttg, but ttg has not been provided!')
    else:
        n_time = test_loader.dataset.max_len
    if chunksize is None:
        chunksize = n_time

    # Retrieve decoded states and actions for different inference cases
    xypsi_dyn = torch.empty(size=(n_state, n_time), device=device).float()
    dv_dyn = torch.empty(size=(n_action, n_time), device=device).float()
    states_dyn = torch.empty(size=(1, n_time, n_state), device=device).float()
    actions_dyn = torch.zeros(size=(1, n_time, n_action), device=device).float()
    rtgs_dyn = torch.empty(size=(1, n_time, 1), device=device).float()
    if test_loader.dataset.mdp_constr:
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
    if test_loader.dataset.mdp_constr:
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
        if (ttg is None) and (test_loader.dataset.mdp_constr):
            with torch.no_grad():
                _, _, ttg_preds_dyn = model(
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
            ttg_dyn_t = ttg_preds_dyn[0,-1] ######## camabiare indice per ttg_preds_ol e analoghi
            ttgs_dyn[:,t,:] = ttg_preds_dyn[0,-1]
            ttgs_pred_dyn[:,t] = (ttg_dyn_t * (data_stats['ttgs_std'][t]+1e-6)) + data_stats['ttgs_mean'][t]
        
        # Compute action pred for dynamics model
        with torch.no_grad():
            _, action_preds_dyn, _ = model(
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

############################################################
def torch_model_inference_ol(model, test_loader, data_sample, rtg_perc=1., ctg_perc=1., rtg=None, ctg_clipped=True):
    # Get dimensions and statistics from the dataset
    n_state = test_loader.dataset.n_state
    n_time = test_loader.dataset.max_len
    n_action = test_loader.dataset.n_action
    data_stats = copy.deepcopy(test_loader.dataset.data_stats)
    data_stats['states_mean'] = data_stats['states_mean'].float().to(device)
    data_stats['states_std'] = data_stats['states_std'].float().to(device)
    data_stats['actions_mean'] = data_stats['actions_mean'].float().to(device)
    data_stats['actions_std'] = data_stats['actions_std'].float().to(device)
    data_stats['goal_mean'] = data_stats['goal_mean'].float().to(device)
    data_stats['goal_std'] = data_stats['goal_std'].float().to(device)

    # Unnormalize the data sample and compute orbital period (data sample is composed by tensors on the cpu)
    if test_loader.dataset.mdp_constr:
        states_i, actions_i, rtgs_i, ctgs_i, goal_i, timesteps_i, attention_mask_i, dt, time_sec, ix = data_sample
        ctgs_i = ctgs_i.view(1, n_time, 1)
    else:
        states_i, actions_i, rtgs_i, goal_i, timesteps_i, attention_mask_i, dt, time_sec, ix = data_sample
    states_i = states_i.to(device)
    rtgs_i = rtgs_i.to(device)
    goal_i = goal_i.to(device)
    timesteps_i = timesteps_i.long().to(device)
    attention_mask_i = attention_mask_i.long().to(device)
    dt = dt.item()
    time_sec = np.array(time_sec[0])
    obs_pos, obs_rad = torch.tensor(np.copy(obs['position'])).to(device), torch.tensor(np.copy(obs['radius'])).to(device)
    obs_rad = (obs_rad + robot_radius)*safety_margin

    # Retrieve decoded states and actions for different inference cases
    xypsi_ol = torch.empty(size=(n_state, n_time), device=device).float()
    dv_ol = torch.empty(size=(n_action, n_time), device=device).float()
    states_ol = torch.empty(size=(1, n_time, n_state), device=device).float()
    actions_ol = torch.zeros(size=(1, n_time, n_action), device=device).float()
    rtgs_ol = torch.empty(size=(1, n_time, 1), device=device).float()
    if test_loader.dataset.mdp_constr:
        ctgs_ol = torch.empty(size=(1, n_time, 1), device=device).float()
    
    runtime0_DT = time.time()
    # Open-loop initialization
    states_ol[:,0,:] = states_i[:,0,:]
    if rtg is None:
        rtgs_ol[:,0,:] = rtgs_i[:,0,:]*rtg_perc
    else:
        rtgs_ol[:,0,:] = rtg
    if test_loader.dataset.mdp_constr:
        ctgs_ol[:,0,:] = ctgs_i[:,0,:]*ctg_perc

    xypsi_ol[:, 0] = (states_ol[:,0,:] * data_stats['states_std'][0]) + data_stats['states_mean'][0]

    # For loop trajectory generation
    for t in np.arange(n_time):

        ##### Open-loop inference
        # Compute action pred for open-loop model
        with torch.no_grad():
            if test_loader.dataset.mdp_constr:
                output_ol = model(
                    states=states_ol[:,:t+1,:],
                    actions=actions_ol[:,:t+1,:],
                    goal=goal_i[:,:t+1,:],
                    returns_to_go=rtgs_ol[:,:t+1,:],
                    constraints_to_go=ctgs_ol[:,:t+1,:],
                    timesteps=timesteps_i[:,:t+1],
                    attention_mask=attention_mask_i[:,:t+1],
                    return_dict=False,
                )
                (_, action_preds_ol) = output_ol
            else:
                output_ol = model(
                    states=states_ol[:,:t+1,:],
                    actions=actions_ol[:,:t+1,:],
                    goal=goal_i[:,:t+1,:],
                    returns_to_go=rtgs_ol[:,:t+1,:],
                    timesteps=timesteps_i[:,:t+1],
                    attention_mask=attention_mask_i[:,:t+1],
                    return_dict=False,
                )
                (_, action_preds_ol, _) = output_ol

        action_ol_t = action_preds_ol[0,t]
        actions_ol[:,t,:] = action_ol_t
        dv_ol[:, t] = (action_ol_t * (data_stats['actions_std'][t]+1e-6)) + data_stats['actions_mean'][t]

        # Compute states pred for open-loop model
        with torch.no_grad():
            if test_loader.dataset.mdp_constr:
                output_ol = model(
                    states=states_ol[:,:t+1,:],
                    actions=actions_ol[:,:t+1,:],
                    goal=goal_i[:,:t+1,:],
                    returns_to_go=rtgs_ol[:,:t+1,:],
                    constraints_to_go=ctgs_ol[:,:t+1,:],
                    timesteps=timesteps_i[:,:t+1],
                    attention_mask=attention_mask_i[:,:t+1],
                    return_dict=False,
                )
                (state_preds_ol, _) = output_ol
            else:
                output_ol = model(
                    states=states_ol[:,:t+1,:],
                    actions=actions_ol[:,:t+1,:],
                    goal=goal_i[:,:t+1,:],
                    returns_to_go=rtgs_ol[:,:t+1,:],
                    timesteps=timesteps_i[:,:t+1],
                    attention_mask=attention_mask_i[:,:t+1],
                    return_dict=False,
                )
                (state_preds_ol, _, _) = output_ol

        state_ol_t = state_preds_ol[0,t]

        # Open-loop propagation of state variable
        if t != n_time-1:
            states_ol[:,t+1,:] = state_ol_t
            xypsi_ol[:,t+1] = (state_ol_t * data_stats['states_std'][t+1]) + data_stats['states_mean'][t+1]

            if test_loader.dataset.mdp_constr:
                reward_ol_t = - torch.linalg.norm(dv_ol[:, t], ord=1)
                rtgs_ol[:,t+1,:] = rtgs_ol[0,t] - reward_ol_t
                viol_ol = torch_check_koz_constraint(xypsi_ol[:,t+1], obs_pos, obs_rad)
                ctgs_ol[:,t+1,:] = ctgs_ol[0,t] - (viol_ol if (not ctg_clipped) else 0)
            else:
                rtgs_ol[:,t+1,:] = rtgs_i[0,t+1]
            actions_ol[:,t+1,:] = 0

    # Pack trajectory's data in a dictionary and compute runtime
    runtime1_DT = time.time()
    runtime_DT = runtime1_DT - runtime0_DT
    DT_trajectory = {
        'xypsi_ol' : xypsi_ol.cpu().numpy(),
        'dv_ol' : dv_ol.cpu().numpy(),
        'time' : time_sec
    }

    return DT_trajectory, runtime_DT

def plot_DT_trajectory(DT_trajectory, plot_orb_time = False, savefig = False, plot_dir = ''):
    # Trajectory data extraction
    xypsi_true = DT_trajectory['xypsi_true']
    xypsi_dyn = DT_trajectory['xypsi_dyn']
    xypsi_ol = DT_trajectory['xypsi_ol']
    dv_true = DT_trajectory['dv_true']
    dv_dyn = DT_trajectory['dv_dyn']
    dv_ol = DT_trajectory['dv_ol']
    ttgs_true = DT_trajectory['ttgs_true']
    ttgs_ol = DT_trajectory['ttgs_ol']
    ttgs_dyn = DT_trajectory['ttgs_dyn']
    time_sec = DT_trajectory['time']
    i = 0
    idx_pred = 0
    idx_plt = 0
    
    # position trajectory
    fig1 = plt.figure()
    ax1 = fig1.add_subplot()
    p01 = ax1.plot(xypsi_true[0,:], xypsi_true[1,:], 'k-', linewidth=1.5, label='true', zorder=3)
    p02 = ax1.plot(xypsi_ol[0,:], xypsi_ol[1,:], 'b-', linewidth=1.5, label='pred o.l.', zorder=3)
    p03 = ax1.plot(xypsi_dyn[0,:], xypsi_dyn[1,:], 'g-', linewidth=1.5, label='pred dyn.', zorder=3)
    p1 = ax1.scatter(xypsi_true[0,0], xypsi_true[1,0], marker = 'o', linewidth=1.5, label='$t_0$', zorder=3)
    p2 = ax1.scatter(xypsi_true[0,-1], xypsi_true[1,-1], marker = '*', linewidth=1.5, label='$t_f$', zorder=3)
    #p3 = ax.scatter(xypsi_true[0,context2.shape[1]//9], xypsi_true[1,context2.shape[1]//9], xypsi_true[2,context2.shape[1]//9], marker = '*', linewidth=1.5, label='$t_{init}$')
    ax1.add_patch(Rectangle((0,0), table['xy_up'][0], table['xy_up'][1], fc=(0.5,0.5,0.5,0.2), ec='k', label='table', zorder=2.5))
    for n_obs in range(obs['radius'].shape[0]):
        label_obs = 'obs' if n_obs == 0 else None
        label_robot = 'robot radius' if n_obs == 0 else None
        ax1.add_patch(Circle(obs['position'][n_obs,:], obs['radius'][n_obs], fc='r', label=label_obs, zorder=2.5))
        ax1.add_patch(Circle(obs['position'][n_obs,:], obs['radius'][n_obs]+robot_radius, fc='r', alpha=0.2, label=label_robot, zorder=2.5))
    ax1.set_xlabel('X [m]', fontsize=10)
    ax1.set_ylabel('Y [m]', fontsize=10)
    ax1.grid(True)
    ax1.legend(loc='best', fontsize=10)
    if savefig and i==idx_pred:
        plt.savefig(plot_dir + f'pos_{idx_plt}.png')
    plt.show()

    plt.figure(figsize=(20,5))
    for j in range(3):
        plt.subplot(1,3,j+1)
        plt.plot(time_sec[0,:len(xypsi_true[j,:])], xypsi_true[j,:], 'k-', linewidth=1.5, label='true')
        plt.plot(time_sec[0,:len(xypsi_ol[j,:])], xypsi_ol[j,:], 'b-', linewidth=1.5, label='pred o.l.')
        #plt.vlines(time_sec[0][(context2.shape[1]//9)+1], np.min(xypsi_ol[j,:]), np.max(xypsi_ol[j,:]), label='t_{init}', linewidth=2, color='red')
        plt.plot(time_sec[0,:len(xypsi_dyn[j,:])], xypsi_dyn[j,:], 'g-', linewidth=1.5, label='pred dyn')
        if j == 0:
            plt.xlabel('time [orbits]' if plot_orb_time else 'time [steps]', fontsize=10)
            plt.ylabel('$ \delta r_r$ [m]', fontsize=10)
            plt.grid(True)
            plt.legend(loc='best', fontsize=10)
        elif j == 1:
            plt.xlabel('time [orbits]' if plot_orb_time else 'time [steps]', fontsize=10)
            plt.ylabel('$\delta r_t$ [m]', fontsize=10)
            plt.grid(True)
            plt.legend(loc='best', fontsize=10)
        elif j == 2:
            plt.xlabel('time [orbits]' if plot_orb_time else 'time [steps]', fontsize=10)
            plt.ylabel('$\delta r_n$ [m]', fontsize=10)
            plt.grid(True)
            plt.legend(loc='best', fontsize=10)
    if savefig and i==idx_pred:
        plt.savefig(plot_dir + f'rtn_pos_{idx_plt}.png')
    plt.show()

    # velocity vs time
    plt.figure(figsize=(20,5))
    for j in range(3):
        plt.subplot(1,3,j+1)
        plt.plot(time_sec[0,:len(xypsi_true[j+3,:])], xypsi_true[j+3,:], 'k-', linewidth=1.5, label='true')
        plt.plot(time_sec[0,:len(xypsi_ol[j+3,:])], xypsi_ol[j+3,:], 'b-', linewidth=1.5, label='pred o.l.')
        #plt.vlines(time_sec[0][(context2.shape[1]//9)+1], np.min(xypsi_ol[j+3,:]), np.max(xypsi_ol[j+3,:]), label='t_{init}', linewidth=2, color='red')
        plt.plot(time_sec[0,:len(xypsi_dyn[j+3,:])], xypsi_dyn[j+3,:], 'g-', linewidth=1.5, label='pred dyn')
        if j == 0:
            plt.xlabel('time [orbits]' if plot_orb_time else 'time [steps]', fontsize=10)
            plt.ylabel('$ \delta v_r$ [m/s]', fontsize=10)
            plt.grid(True)
            plt.legend(loc='best', fontsize=10)
        elif j == 1:
            plt.xlabel('time [orbits]' if plot_orb_time else 'time [steps]', fontsize=10)
            plt.ylabel('$\delta v_t$ [m/s]', fontsize=10)
            plt.grid(True)
            plt.legend(loc='best', fontsize=10)
        elif j == 2:
            plt.xlabel('time [orbits]' if plot_orb_time else 'time [steps]', fontsize=10)
            plt.ylabel('$\delta v_n$ [m/s]', fontsize=10)
            plt.grid(True)
            plt.legend(loc='best', fontsize=10)
    if savefig and i==idx_pred:
        plt.savefig(plot_dir + f'rtn_vel_{idx_plt}.png')
    plt.show()
    ###### DELTA-V

    # components
    plt.figure(figsize=(20,5))
    for j in range(3):
        plt.subplot(1,3,j+1)
        plt.stem(time_sec[0,:len(dv_true[j,:])], dv_true[j,:]*1000., 'k-', label='true')
        plt.stem(time_sec[0,:len(dv_ol[j,:])], dv_ol[j,:]*1000., 'b-', label='pred o.l.')
        plt.stem(time_sec[0,:len(dv_dyn[j,:])], dv_dyn[j,:]*1000., 'g-', label='pred dyn.')
        #plt.vlines(time_sec[0][(context2.shape[1]//9)+1], np.min(dv_ol[j,:]*1000.), np.max(dv_ol[j,:]*1000.), label='t_{init}', linewidth=2, color='red')
        if j == 0:
            plt.xlabel('time [orbits]' if plot_orb_time else 'time [steps]', fontsize=10)
            plt.ylabel('$ \Delta \delta v_r$ [mm/s]', fontsize=10)
            plt.grid(True)
            plt.legend(loc='best', fontsize=10)
        elif j == 1:
            plt.xlabel('time [orbits]' if plot_orb_time else 'time [steps]', fontsize=10)
            plt.ylabel('$ \Delta \delta v_t$ [mm/s]', fontsize=10)
            plt.grid(True)
            plt.legend(loc='best', fontsize=10)
        elif j == 2:
            plt.xlabel('time [orbits]' if plot_orb_time else 'time [steps]', fontsize=10)
            plt.ylabel('$ \Delta \delta v_n$ [mm/s]', fontsize=10)
            plt.grid(True)
            plt.legend(loc='best', fontsize=10)
    if savefig and i==idx_pred:
        plt.savefig(plot_dir + f'delta_v_{idx_plt}.png')
    plt.show()

    # norm
    plt.figure()
    plt.stem(time_sec[0,:len(dv_true[0,:])], la.norm(dv_true*1000., axis=0), 'k-', label='true')
    plt.stem(time_sec[0,:len(dv_ol[0,:])], la.norm(dv_ol*1000., axis=0), 'b-', label='pred o.l.')
    plt.stem(time_sec[0,:len(dv_dyn[0,:])], la.norm(dv_dyn*1000., axis=0), 'g-', label='pred dyn')
    plt.xlabel('time [orbits]' if plot_orb_time else 'time [steps]', fontsize=10)
    plt.ylabel('$ || \Delta \delta v || $ [mm/s]', fontsize=10)
    plt.grid(True)
    plt.legend(loc='best', fontsize=10)
    if savefig and i==idx_pred:
        plt.savefig(plot_dir + f'delta_v_norm_{idx_plt}.png')
    plt.show()

    # time to go
    plt.figure()
    plt.plot(time_sec[0,:len(ttgs_true[0])], ttgs_true[0], 'k-', label='true')
    plt.plot(time_sec[0,:len(ttgs_ol[0])], ttgs_ol[0], 'b-', label='pred o.l.')
    plt.plot(time_sec[0,:len(ttgs_dyn[0])], ttgs_dyn[0], 'g-', label='pred dyn')
    plt.xlabel('time [orbits]' if plot_orb_time else 'time [steps]', fontsize=10)
    plt.ylabel('Time-to-go [s]', fontsize=10)
    plt.grid(True)
    plt.legend(loc='best', fontsize=10)
    if savefig and i==idx_pred:
        plt.savefig(plot_dir + f'ttgs_{idx_plt}.png')
    plt.show()
