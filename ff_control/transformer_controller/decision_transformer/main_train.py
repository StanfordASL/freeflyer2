import os
import sys
root_folder = os.path.abspath(os.path.dirname(os.getcwd()))
sys.path.append(root_folder)
import numpy as np
import numpy.linalg as la
import matplotlib.pyplot as plt
from transformers import DecisionTransformerConfig
from decision_transformer.art import AutonomousFreeflyerTransformer, AutonomousFreeflyerTransformer_pred_time
import torch
import decision_transformer.manage as TTO_manager
from decision_transformer.manage import device
from optimization.ff_scenario import generalized_time, generalized_obs

# Initial parameters
model_name_4_saving = 'checkpoint_ff_time40_100_chunk100R_ctgrtg'
model_config = TTO_manager.transformer_import_config(model_name_4_saving)
print('Training specs: \n Model name:', model_name_4_saving, '\n Dataset scenario:', model_config['dataset_scenario'],
      '\n Chunksize:', model_config['chunksize'], '\n Random chunk:', model_config['random_chunk'])
datasets, dataloaders = TTO_manager.get_train_val_test_data(mdp_constr=model_config['mdp_constr'], dataset_scenario=model_config['dataset_scenario'],
                                                            timestep_norm=model_config['timestep_norm'], chunksize=model_config['chunksize'],
                                                            random_chunk=model_config['random_chunk'])
train_loader, eval_loader, test_loader = dataloaders
n_state = train_loader.dataset.n_state
n_data = train_loader.dataset.n_data
n_action = train_loader.dataset.n_action
n_time = train_loader.dataset.max_len
data_stats = train_loader.dataset.data_stats

# Transformer parameters
config = DecisionTransformerConfig(
    state_dim=n_state, 
    act_dim=n_action,
    hidden_size=384,
    max_ep_len=n_time,
    vocab_size=1,
    action_tanh=False,
    n_positions=2048 if generalized_time else 1024,
    n_layer=6,
    n_head=6,
    n_inner=None,
    resid_pdrop=0.1,
    embd_pdrop=0.1,
    attn_pdrop=0.1,
    )
if generalized_time and (not generalized_obs):
    model = AutonomousFreeflyerTransformer_pred_time(config)
elif (not generalized_obs) and (not generalized_time):
    model = AutonomousFreeflyerTransformer(config)
else:
    raise NameError('Generalized obstacles not implemented yet!')

model_size = sum(t.numel() for t in model.parameters())
print(f"GPT size: {model_size/1000**2:.1f}M parameters")
model.to(device);

from torch.optim import AdamW
from accelerate import Accelerator
from transformers import get_scheduler
optimizer = AdamW(model.parameters(), lr=3e-5)
accelerator = Accelerator(mixed_precision='no', gradient_accumulation_steps=8)
model, optimizer, train_dataloader, eval_dataloader = accelerator.prepare(
    model, optimizer, train_loader, eval_loader
)
num_train_epochs = 1
num_update_steps_per_epoch = len(train_dataloader)
num_training_steps = 10000000000

lr_scheduler = get_scheduler(
    name="linear",
    optimizer=optimizer,
    num_warmup_steps=10,
    num_training_steps=num_training_steps,
)

# To activate only when starting from a pretrained model
# accelerator.load_state(root_folder + '/decision_transformer/saved_files/checkpoints/' + model_name_4_saving)

if generalized_time:
    # Eval function
    eval_iters = 100
    ttg_zero_norm = (0.01 - data_stats['ttgs_mean'][0]) / (data_stats['ttgs_std'][0] + 1e-6)
    @torch.no_grad()
    def evaluate():
        model.eval()
        losses = []
        losses_state = []
        losses_action = []
        losses_ttgs = []
        for step in range(eval_iters):
            data_iter = iter(eval_dataloader)
            states_i, actions_i, rtgs_i, ctgs_i, ttgs_i, goal_i, timesteps_i, attention_mask_i, _, _, _ = next(data_iter)
            mask = ttgs_i > ttg_zero_norm#rtgs_i < 0
            mask_act = torch.repeat_interleave(mask, 3, 2)
            mask_st = torch.repeat_interleave(mask[:,1:,:], 6, 2)
            with torch.no_grad():
                state_preds, action_preds, ttg_preds = model(
                    states=states_i,
                    actions=actions_i,
                    goal=goal_i,
                    returns_to_go=rtgs_i,
                    constraints_to_go=ctgs_i,
                    times_to_go=ttgs_i,
                    timesteps=timesteps_i,
                    attention_mask=attention_mask_i,
                    return_dict=False,
                )
            loss_i = torch.mean((action_preds[mask_act] - actions_i[mask_act]) ** 2)#torch.mean((action_preds - actions_i) ** 2)#
            loss_i_state = torch.mean((state_preds[:,:-1,:][mask_st] - states_i[:,1:,:][mask_st]) ** 2)#torch.mean((state_preds[:,:-1,:] - states_i[:,1:,:]) ** 2)#
            loss_i_ttgs = torch.mean((ttg_preds[mask] - ttgs_i[mask]) ** 2)#torch.mean((ttgs_pred - ttgs_i) ** 2)#
            losses.append(accelerator.gather(loss_i + loss_i_state + loss_i_ttgs))
            losses_state.append(accelerator.gather(loss_i_state))
            losses_action.append(accelerator.gather(loss_i))
            losses_ttgs.append(accelerator.gather(loss_i_ttgs))
        loss = torch.mean(torch.tensor(losses))
        loss_state = torch.mean(torch.tensor(losses_state))
        loss_action = torch.mean(torch.tensor(losses_action))
        loss_ttgs = torch.mean(torch.tensor(losses_ttgs))
        model.train()
        return loss.item(), loss_state.item(), loss_action.item(), loss_ttgs.item()

    eval_loss, loss_state, loss_action, loss_ttg = evaluate()
    accelerator.print({"loss/eval": eval_loss, "loss/state": loss_state, "loss/action": loss_action, "loss/ttg": loss_ttg})

    # Training

    eval_steps = 500
    samples_per_step = accelerator.state.num_processes * train_loader.batch_size
    torch.manual_seed(4)

    model.train()
    completed_steps = 0
    log = {
        'loss':[],
        'loss_state':[],
        'loss_action':[],
        'loss_ttg' : []
    }
    '''log = np.load(root_folder + '/decision_transformer/saved_files/checkpoints/' + model_name_4_saving + '/log.npz', allow_pickle=True)['log'].item()'''
    for epoch in range(num_train_epochs):
        for step, batch in enumerate(train_dataloader, start=0):
            with accelerator.accumulate(model):
                states_i, actions_i, rtgs_i, ctgs_i, ttgs_i, goal_i, timesteps_i, attention_mask_i, _, _, _ = batch
                mask = ttgs_i > ttg_zero_norm#rtgs_i < 0
                mask_act = torch.repeat_interleave(mask, 3, 2)
                mask_st = torch.repeat_interleave(mask[:,1:,:], 6, 2)
                state_preds, action_preds, ttg_preds = model(
                    states=states_i,
                    actions=actions_i,
                    goal=goal_i,
                    returns_to_go=rtgs_i,
                    constraints_to_go=ctgs_i,
                    times_to_go=ttgs_i,
                    timesteps=timesteps_i,
                    attention_mask=attention_mask_i,
                    return_dict=False,
                )
                loss_i_action = torch.mean((action_preds[mask_act] - actions_i[mask_act]) ** 2)#torch.mean((action_preds - actions_i) ** 2)#
                loss_i_state = torch.mean((state_preds[:,:-1,:][mask_st] - states_i[:,1:,:][mask_st]) ** 2)#torch.mean((state_preds[:,:-1,:] - states_i[:,1:,:]) ** 2)#
                loss_i_ttg = torch.mean((ttg_preds[mask] - ttgs_i[mask]) ** 2)#torch.mean((ttgs_pred - ttgs_i) ** 2)#
                loss = loss_i_action + loss_i_state + loss_i_ttg
                if step % 100 == 0:
                    accelerator.print(
                        {
                            "lr": lr_scheduler.get_lr(),
                            "samples": step * samples_per_step,
                            "steps": completed_steps,
                            "loss/train": loss.item(),
                        }
                    )
                accelerator.backward(loss)
                accelerator.clip_grad_norm_(model.parameters(), 1.0)
                optimizer.step()
                lr_scheduler.step()
                optimizer.zero_grad()
                completed_steps += 1
                if (step % (eval_steps)) == 0:
                    eval_loss, loss_state, loss_action, loss_ttg = evaluate()
                    accelerator.print({"loss/eval": eval_loss, "loss/state": loss_state, "loss/action": loss_action, "loss/ttg": loss_ttg})
                    log['loss'].append(eval_loss)
                    log['loss_state'].append(loss_state)
                    log['loss_action'].append(loss_action)
                    log['loss_ttg'].append(loss_ttg)
                    model.train()
                    accelerator.wait_for_everyone()
                if (step % (eval_steps*10)) == 0:
                    print('Saving model..')
                    accelerator.save_state(root_folder+'/decision_transformer/saved_files/checkpoints/'+model_name_4_saving)
                    np.savez_compressed(root_folder + '/decision_transformer/saved_files/checkpoints/' +model_name_4_saving+ '/log',
                                log = log
                                )
elif (not generalized_time) and (not generalized_obs):
    # Eval function
    eval_iters = 100
    @torch.no_grad()
    def evaluate():
        model.eval()
        losses = []
        losses_state = []
        losses_action = []
        for step in range(eval_iters):
            data_iter = iter(eval_dataloader)
            states_i, actions_i, rtgs_i, ctgs_i, goal_i, timesteps_i, attention_mask_i, _, _, _ = next(data_iter)
            with torch.no_grad():
                state_preds, action_preds = model(
                    states=states_i,
                    actions=actions_i,
                    goal=goal_i,
                    returns_to_go=rtgs_i,
                    constraints_to_go=ctgs_i,
                    timesteps=timesteps_i,
                    attention_mask=attention_mask_i,
                    return_dict=False,
                )
            loss_i = torch.mean((action_preds - actions_i) ** 2)
            loss_i_state = torch.mean((state_preds[:,:-1,:] - states_i[:,1:,:]) ** 2)
            losses.append(accelerator.gather(loss_i + loss_i_state))
            losses_state.append(accelerator.gather(loss_i_state))
            losses_action.append(accelerator.gather(loss_i))
        loss = torch.mean(torch.tensor(losses))
        loss_state = torch.mean(torch.tensor(losses_state))
        loss_action = torch.mean(torch.tensor(losses_action))
        model.train()
        return loss.item(), loss_state.item(), loss_action.item()

    eval_loss, loss_state, loss_action = evaluate()
    accelerator.print({"loss/eval": eval_loss, "loss/state": loss_state, "loss/action": loss_action})

    # Training

    eval_steps = 500
    samples_per_step = accelerator.state.num_processes * train_loader.batch_size
    torch.manual_seed(4)

    model.train()
    completed_steps = 0
    log = {
        'loss':[],
        'loss_state':[],
        'loss_action':[]
    }
    '''log = np.load(root_folder + '/decision_transformer/saved_files/checkpoints/' + model_name_4_saving + '/log.npz', allow_pickle=True)['log'].item()'''
    for epoch in range(num_train_epochs):
        for step, batch in enumerate(train_dataloader, start=0):
            with accelerator.accumulate(model):
                states_i, actions_i, rtgs_i, ctgs_i, goal_i, timesteps_i, attention_mask_i, _, _, _ = batch
                state_preds, action_preds = model(
                    states=states_i,
                    actions=actions_i,
                    goal=goal_i,
                    returns_to_go=rtgs_i,
                    constraints_to_go=ctgs_i,
                    timesteps=timesteps_i,
                    attention_mask=attention_mask_i,
                    return_dict=False,
                )
                loss_i_action = torch.mean((action_preds - actions_i) ** 2)
                loss_i_state = torch.mean((state_preds[:,:-1,:] - states_i[:,1:,:]) ** 2)
                loss = loss_i_action + loss_i_state
                if step % 100 == 0:
                    accelerator.print(
                        {
                            "lr": lr_scheduler.get_lr(),
                            "samples": step * samples_per_step,
                            "steps": completed_steps,
                            "loss/train": loss.item(),
                        }
                    )
                accelerator.backward(loss)
                accelerator.clip_grad_norm_(model.parameters(), 1.0)
                optimizer.step()
                lr_scheduler.step()
                optimizer.zero_grad()
                completed_steps += 1
                if (step % (eval_steps)) == 0:
                    eval_loss, loss_state, loss_action = evaluate()
                    accelerator.print({"loss/eval": eval_loss, "loss/state": loss_state, "loss/action": loss_action})
                    log['loss'].append(eval_loss)
                    log['loss_state'].append(loss_state)
                    log['loss_action'].append(loss_action)
                    model.train()
                    accelerator.wait_for_everyone()
                if (step % (eval_steps*10)) == 0:
                    print('Saving model..')
                    accelerator.save_state(root_folder+'/decision_transformer/saved_files/checkpoints/'+model_name_4_saving)
                    np.savez_compressed(root_folder + '/decision_transformer/saved_files/checkpoints/' +model_name_4_saving+ '/log',
                                log = log
                                )