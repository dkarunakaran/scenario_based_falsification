#!/usr/bin/env python

from __future__ import division
import os
import json
import matplotlib

# For the error: Exception ignored in: <bound method Image.del of <tkinter.PhotoImage object at 0x7f1b5f86a710>> Traceback (most recent call last):
matplotlib.use('Agg')
from mpl_toolkits import mplot3d
from matplotlib import pyplot as plt

import numpy as np

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
failure_rss_data = evaluation['rss_data_per_episode'][len(evaluation['action_per_episode'])-1]
pick_an_episode = 0
failure_scenario = failure_scenario_data[pick_an_episode]
failure_scenario_reward = failure_reward_data[pick_an_episode]
failure_scenario_action = failure_action_data[pick_an_episode]
failure_scenario_action = failure_scenario_action.split("_")
failure_rss = failure_rss_data[pick_an_episode]

print(failure_scenario_action)


