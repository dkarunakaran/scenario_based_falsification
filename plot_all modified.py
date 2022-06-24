#!/usr/bin/env python

from __future__ import division
import os
import json
import matplotlib

# For the error: Exception ignored in: <bound method Image.del of <tkinter.PhotoImage object at 0x7f1b5f86a710>> Traceback (most recent call last):
matplotlib.use('Agg')
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import pyplot as plt
from random import randint
from collections import deque
import numpy as np
import scipy
import scipy.stats
from scipy.stats import gaussian_kde


class PDF:
    
    def __init__(self,):
        self.dist_names = ['gamma', 'norm','lognorm','expon','pareto', 'weibull_min', 'weibull_max', 'uniform', 'exponnorm']
        self.dist_results = []
        self.params = {}
        
        self.d_name = ""
        self.p_value = 0
        self.param = None
        
        self.is_fitted = False

    def select_fit(self, data):
        self.dist_results = []
        self.params = {}
        for dist_name in self.dist_names:
            dist = getattr(scipy.stats, dist_name)
            param = dist.fit(data)
            
            self.params[dist_name] = param
            # Applying the Kolmogorov-Smirnov test to compare a sample with a reference probability distribution 
            D, p = scipy.stats.kstest(data, dist_name, args=param);
            self.dist_results.append((dist_name,p))

        # Choose the best fitted distribution
        sel_dist,p = (max(self.dist_results,key=lambda item:item[1]))
        
        # store the name of the best fitted distribution and its p value
        self.d_name = sel_dist
        self.p_value = p
        
        self.is_fitted = True
        return self.d_name,self.p_value

    
    def random(self, n = 1):
        if self.is_fitted:
            dist_name = self.d_name
            param = self.params[dist_name]

            #initiate the scipy distribution
            dist = getattr(scipy.stats, dist_name)
            return dist.rvs(*param[:-2], loc=param[-2], scale=param[-1], size=n)
        else:
            raise ValueError('Must first run the Fit method.')

    def kde_fit(self, x = None, slimit = 0, elimit = 10, sample_size = 200):
        density = gaussian_kde(x)
        xs = np.linspace(slimit, elimit, sample_size)
        density.covariance_factor = lambda : .25
        density._compute_covariance()

        return xs, density(xs)


pdf = PDF()

path_to_dir = '/parameter_optimisation_rl/save_dir/plot_data/'
path_to_save_dir = '/parameter_optimisation_rl/save_dir/plots/'
dirFiles = os.listdir(path_to_dir)
file_list = []
for _file in dirFiles: #filter out all non jpgs
    if 'plot_' in _file:
        file_list.append(_file)
file_list.sort(key=lambda f: int(filter(str.isdigit, f)))      
evaluation = {
    'avg_exploration': [], 
    'episode':[], 
    'avg_reward': [], 
    'loss_per_batch': [], 
    'no_explored_step_batch': [], 
    'rss_failure': None,
    'scenario_data_per_episode': [],
    'reward_data_per_episode': [],
    'action_per_episode': [],
    'rss_data_per_episode': []
    }
episode = 0
pick_rss_file_no = 0
pick_rss_episode = 1
file_count = 0
for _file in file_list:

    if _file == 'plot_0':
        print("skipping plot_0")
        continue

     # Each file iteration
    with open(path_to_dir+_file) as json_file:

        data = json.load(json_file)
        
        # Average exploration
        avg_exploration = sum(data['exploration'])/len(data['exploration'])
        episode = data['episode']
        avg_reward = sum(data['reward'])/len(data['reward'])

        if file_count == pick_rss_file_no:
            rss_data = data['rss_data_per_episode'][pick_rss_episode]

    evaluation["avg_exploration"].append(avg_exploration)
    evaluation["avg_reward"].append(avg_reward)
    evaluation['episode'].append(episode)
    evaluation['loss_per_batch'].append(sum(data['loss_per_episode'])/25)
    evaluation['no_explored_step_batch'].append(data['no_explored_step_batch'])
    evaluation['rss_failure'] = rss_data
    evaluation['rss_data_per_episode'].append(data['rss_data_per_episode'])
    evaluation['scenario_data_per_episode'].append(data['scenario_data_per_episode'])
    evaluation['reward_data_per_episode'].append(data['reward'])
    evaluation['action_per_episode'].append(data['action_per_episode'])
    

    file_count += 1

failure_scenario_data = evaluation['scenario_data_per_episode'][len(evaluation['scenario_data_per_episode'])-1]
failure_reward_data = evaluation['reward_data_per_episode'][len(evaluation['reward_data_per_episode'])-1]
failure_action_data = evaluation['action_per_episode'][len(evaluation['action_per_episode'])-1]
pick_an_episode = 0
failure_scenario = failure_scenario_data[pick_an_episode]
failure_scenario_reward = failure_reward_data[pick_an_episode]
failure_scenario_action = failure_action_data[pick_an_episode]
failure_scenario_action = failure_scenario_action.split("_")
#print(failure_scenario_reward)
#pick_an_episode = 10
pick_an_episode = 4
pick_a_plot_file = 0
success_scenario_data = evaluation['scenario_data_per_episode'][pick_a_plot_file]
success_reward_data = evaluation['reward_data_per_episode'][pick_a_plot_file]
success_action_data = evaluation['action_per_episode'][pick_a_plot_file]
sucess_rss_data = evaluation['rss_data_per_episode'][pick_a_plot_file]
success_scenario = success_scenario_data[pick_an_episode]
success_scenario_reward = success_reward_data[pick_an_episode]
success_scenario_action = success_action_data[pick_an_episode]
success_scenario_action = success_scenario_action.split("_")
sucess_rss = sucess_rss_data[pick_an_episode]

def modified_timestep_fn(scenario = None):
    start = False
    total_timesteps = 0
    modifed_timestep = 0
    for timestep in range(0, len(scenario['speed_ego'])) :
        if start is False and scenario['speed_ego'][timestep] < 0.05:
            start = True

        if start and scenario['speed_ego'][timestep] > 0.05:
            break
        total_timesteps += 1

    modifed_timestep = total_timesteps
    if modifed_timestep == scenario['timesteps']:
        
        # Finding the small distance
        small_dist = None
        small_timesteps = 0
        for timestep in range(0, len(scenario['euclidean_distance'])):
            if small_dist is None:
                small_dist = scenario['euclidean_distance'][timestep]
                small_timesteps = timestep
            
            if scenario['euclidean_distance'][timestep] < small_dist:
                small_dist = scenario['euclidean_distance'][timestep]
                small_timesteps = timestep

        modifed_timestep = small_timesteps

    return modifed_timestep



print("-----------------------------")
print("Existing success data")
#print(success_scenario)
print(success_scenario_reward)
#print(sucess_rss)
print("Modified success data")
modified_success_timestep = modified_timestep_fn(success_scenario)
print("total_timestep: {}".format(success_scenario['timesteps']))
print("Further_modifed_timestep: {}".format(modified_success_timestep))


print("-----------------------------")
print("Existing failure data")
#print(failure_scenario)
print(failure_scenario_reward)
#print(sucess_rss)
print("Modified failure data")
modifed_failure_timestep = modified_timestep_fn(failure_scenario)
print("total_timestep: {}".format(failure_scenario['timesteps']))
print("Further_modifed_timestep: {}".format(modifed_failure_timestep))


# Finding success scenarios
episode_ratio = None
all_rss_success_ratio = []
file_index = 0
episode = 0
ratio_per_limit = 45
small = None
for scenario_data_file in evaluation['scenario_data_per_episode']:
    file_episode_no = 0

    # Each episode in a file
    for episode_item in scenario_data_file:
        
        modified_timestep = modified_timestep_fn(episode_item)
        rss_failure = 0
        for timestep in range(0, modified_timestep):
            if episode_item['timestep_failure'][timestep] is 1:
                rss_failure += 1
        ratio = round((rss_failure/modified_timestep)*100, 2)
        reward = evaluation['reward_data_per_episode'][file_index][file_episode_no]
        if ratio < ratio_per_limit:
            all_rss_success_ratio.append(
                {
                    'file_index': file_index,
                    'file_episode_no': file_episode_no,
                    'episode': episode,
                    'rss_ratio': ratio,
                    'reward': reward
                }
            )
            if small is None:
                small = reward
                episode_ratio = {
                        'file_index': file_index,
                        'file_episode_no': file_episode_no,
                        'episode': episode,
                        'rss_ratio': ratio,
                        'reward': reward
                }

            if reward < small:
                small = reward
                episode_ratio = {
                        'file_index': file_index,
                        'file_episode_no': file_episode_no,
                        'episode': episode,
                        'rss_ratio': ratio,
                        'reward': reward
                }
            
        episode += 1
        file_episode_no +=1
    file_index += 1


print(len(all_rss_success_ratio))
print(episode_ratio)

# Finding failure scenarios
episode_ratio = []
all_rss_failure_ratio = []
file_index = 0
episode = 0
ratio_per_limit = 45
for scenario_data_file in evaluation['scenario_data_per_episode']:
    file_episode_no = 0

    # Each episode in a file
    for episode_item in scenario_data_file:
        modified_timestep = modified_timestep_fn(episode_item)
        rss_failure = 0
        for timestep in range(0, modified_timestep):
            if episode_item['timestep_failure'][timestep] is 1:
                rss_failure += 1
        ratio = round((rss_failure/modified_timestep)*100, 2)
        reward = evaluation['reward_data_per_episode'][file_index][file_episode_no]
        if ratio >= ratio_per_limit:
            all_rss_failure_ratio.append(ratio)
            episode_ratio = {
                    'file_index': file_index,
                    'file_episode_no': file_episode_no,
                    'episode': episode,
                    'rss_ratio': ratio,
                    'reward': reward
            }
            
        episode += 1
        file_episode_no +=1
    file_index += 1

print(len(all_rss_failure_ratio))
print(episode_ratio)


'''
# Each episode in a file
for episode_item in success_scenario:
    rss_failure = 0
    count = 0
    for item in episode_item['timestep_failure']:
        if item == 1:
            rss_failure += 1
        print(episode_item['speed_ego'][count])
        count += 1
'''

plot_main = True
if plot_main:

    xs, ys = pdf.kde_fit(all_rss_failure_ratio, slimit = 45, elimit = 65, sample_size = len(all_rss_failure_ratio))
    name = path_to_save_dir+"kde_fit"
    plt.figure()
    plt.plot(xs, ys) 
    plt.ylabel("Probability")
    plt.xlabel("RSS failure timesteps percentage")
    plt.savefig(name)
    plt.close()


    # exploration decay per episode plot
    legend = []
    name = path_to_save_dir+"exploration_decay_per_episode_plot_iteration"
    plt.figure()
    plt.title("Exploration decay per episode")
    plt.xlabel("Episode")
    plt.ylabel("Avg. exploration")
    plt.plot(evaluation['episode'], evaluation["avg_exploration"], linewidth=2) 
    #plt.legend(legend)
    plt.savefig(name)
    plt.close()

    legend = []
    name = path_to_save_dir+"avg_reward_per_episode"
    plt.figure()
    plt.title("Average reward per episode")
    plt.xlabel("Episode")
    plt.ylabel("Avg. reward")
    plt.plot(evaluation['episode'], evaluation["avg_reward"], linewidth=2) 
    #plt.legend(legend)
    plt.savefig(name)
    plt.close()

    legend = []
    name = path_to_save_dir+"avg_pg_loss_per_batch"
    plt.figure()
    plt.title("Average PG loss per batch")
    plt.xlabel("Episode")
    plt.ylabel("Avg. PG loss")
    plt.plot(evaluation['episode'], evaluation["loss_per_batch"], linewidth=2) 
    #plt.legend(legend)
    plt.savefig(name)
    plt.close()

    legend = []
    name = path_to_save_dir+"no_explored_step_batch"
    plt.figure()
    plt.title("No, of explored actions in a batch")
    plt.xlabel("Episode")
    plt.ylabel("Explored actions")
    plt.plot(evaluation['episode'], evaluation["no_explored_step_batch"], linewidth=2) 
    #plt.legend(legend)
    plt.savefig(name)
    plt.close()

    name = path_to_save_dir+"rss_failure"
    labels = 'Failure timesteps', 'Success timesteps'
    rss_split = evaluation['rss_failure'].split("_")
    sizes = [rss_split[0], rss_split[1]]
    explode = (0.1, 0)

    fig1, ax1 = plt.subplots()
    ax1.pie(sizes, explode=explode, labels=labels, autopct='%1.1f%%',
            shadow=True, startangle=90)
    ax1.axis('equal')  # Equal aspect ratio ensures that pie is drawn as a circle.

    plt.savefig(name)
    plt.close()



    #----------------------FAILURE SCENARIOS------------------------------
   
    rss_failure = 0
    for timestep in range(0, modifed_failure_timestep):
        if failure_scenario['timestep_failure'][timestep] is 1:
            rss_failure += 1

    failure_timestep = rss_failure
    success_timestep = modifed_failure_timestep - failure_timestep

    name = path_to_save_dir+"challening_scenario_timestep_failure"
    labels = 'Failure timesteps', 'Success timesteps'
    sizes = [failure_timestep, success_timestep]
    explode = (0.1, 0)

    fig1, ax1 = plt.subplots()
    ax1.pie(sizes, explode=explode, labels=labels, autopct='%1.1f%%',
            shadow=True, startangle=90)
    ax1.axis('equal')  # Equal aspect ratio ensures that pie is drawn as a circle.

    plt.savefig(name)
    plt.close()


    speed_ped = []
    speed_ego = []
    ts = []
    for timestep in range(0, modifed_failure_timestep):
        ts.append(timestep)
        speed_ped.append(failure_scenario['speed_ped'][timestep])
        speed_ego.append(failure_scenario['speed_ego'][timestep])
    x = ts; y1 = speed_ped; y2 = speed_ego

    plt.figure(figsize=(6,1), dpi=120) # 10 is width, 4 is height
    plt.rcParams.update({'font.size': 22})
    # Plot Line1 (Left Y Axis)
    fig, ax1 = plt.subplots(1,1,figsize=(16,7), dpi= 80)
    fig.suptitle('ego-long-pos = {}, ped-accel = {}, ped-vel = {}, ped-long-pos = {}, weather = {}'.format(
        float(failure_scenario_action[0]), 
        round(float(failure_scenario_action[1]),3), 
        round(float(failure_scenario_action[2]),3), 
        round(float(failure_scenario_action[3]),3), 
        float(failure_scenario_action[4])), 
        fontsize=15
    )
    ax1.plot(x, y1, color='tab:red', label='Pedestrain speed', linewidth=3)
    ax1.plot(x, y2, color='tab:green', label='Ego vehicle speed', linewidth=3)
    ax1.legend(bbox_to_anchor=(0., 0.97, 1., .102),
            ncol=2, mode="expand", borderaxespad=0.,  prop={"size":15})

    # Just Decorations!! -------------------
    # ax1 (left y axis)
    ax1.set_xlabel('Time steps', fontsize=20)
    ax1.set_ylabel('Speed(m/s)')
    ax1.tick_params(axis='y', rotation=0)

    plt.savefig(path_to_save_dir+"challenging_scenario")
    plt.close()


    ts = []
    timestep_failure = []
    for timestep in range(0, modifed_failure_timestep):
        ts.append(timestep)
        timestep_failure.append(failure_scenario['timestep_failure'][timestep])

    plt.figure(figsize=(9,5), dpi=120) # 10 is width, 4 is height
    plt.rcParams.update({'font.size': 15})
    #plt.title("Average action per episode")
    plt.xlabel("Time steps")
    plt.ylabel("Failure")
    plt.plot(ts, timestep_failure, linewidth=2) 
    plt.savefig(path_to_save_dir+"challenging_scenario_timesteps")
    plt.close()

    #----------------------SUCCESS SCENARIOS------------------------------
    
    rss_failure = 0
    for timestep in range(0, modified_success_timestep):
        if success_scenario['timestep_failure'][timestep] is 1:
            rss_failure += 1

    failure_timestep = rss_failure
    success_timestep = modified_success_timestep - failure_timestep

    name = path_to_save_dir+"success_scenario_timestep_failure"
    labels = 'Failure timesteps', 'Success timesteps'
    sizes = [failure_timestep, success_timestep]
    explode = (0.1, 0)

    fig1, ax1 = plt.subplots()
    ax1.pie(sizes, explode=explode, labels=labels, autopct='%1.1f%%',
            shadow=True, startangle=90)
    ax1.axis('equal')  # Equal aspect ratio ensures that pie is drawn as a circle.

    plt.savefig(name)
    plt.close()

    speed_ped = []
    speed_ego = []
    ts = []
    for timestep in range(0, modified_success_timestep):
        ts.append(timestep)
        speed_ped.append(success_scenario['speed_ped'][timestep])
        speed_ego.append(success_scenario['speed_ego'][timestep])
    x = ts; y1 = speed_ped; y2 = speed_ego
    plt.figure(figsize=(6,1), dpi=120) # 10 is width, 4 is height
    plt.rcParams.update({'font.size': 22})
    # Plot Line1 (Left Y Axis)
    fig, ax1 = plt.subplots(1,1,figsize=(16,7), dpi= 80)
    fig.suptitle('ego-long-pos = {}, ped-accel = {}, ped-vel = {}, ped-long-pos = {}, weather = {}'.format(
        float(success_scenario_action[0]), 
        round(float(success_scenario_action[1]),3), 
        round(float(success_scenario_action[2]),3), 
        round(float(success_scenario_action[3]),3), 
        float(success_scenario_action[4])), 
        fontsize=15
    )
    ax1.plot(x, y1, color='tab:red', label='Pedestrain speed', linewidth=3)
    ax1.plot(x, y2, color='tab:green', label='Ego vehicle speed', linewidth=3)
    ax1.legend(bbox_to_anchor=(0., 0.97, 1., .102),
            ncol=2, mode="expand", borderaxespad=0.,  prop={"size":15})

    # Just Decorations!! -------------------
    # ax1 (left y axis)
    ax1.set_xlabel('Time steps', fontsize=20)
    ax1.set_ylabel('Speed(m/s)')
    ax1.tick_params(axis='y', rotation=0)
    plt.savefig(path_to_save_dir+"success_scenario")
    plt.close()

    
    ts = []
    timestep_failure = []
    for timestep in range(0, modified_success_timestep):
        ts.append(timestep)
        timestep_failure.append(success_scenario['timestep_failure'][timestep])

    plt.figure(figsize=(9,5), dpi=120) # 10 is width, 4 is height
    plt.rcParams.update({'font.size': 15})
    #plt.title("Average action per episode")
    plt.xlabel("Time steps")
    plt.ylabel("Failure")
    plt.plot(ts, timestep_failure, linewidth=2) 
    plt.savefig(path_to_save_dir+"success_scenario_timesteps")
    plt.close()

else:
    print("here")
    '''plt.figure()
    hist = plt.hist(all_rss_failure_ratio, bins='auto', color='#B45743', rwidth=0.25, normed=False)
    plt.title('Acceleration histogram')
    plt.xlabel("Acceleration")
    plt.ylabel("Normalised count")
    plt.savefig(path_to_save_dir+"distribution")'''





        
