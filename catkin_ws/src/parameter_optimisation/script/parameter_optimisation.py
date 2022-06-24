#!/usr/bin/env python

from __future__ import division
import rospy
import carla
import time
import json
import os
import pkg_resources
from distutils.version import LooseVersion
from actors.weather import Weather
from enviornment import Enviornment
import sys
from scenario import Scenario
import threading
from pdf import PDF
from controller import Controller
from search_space import SearchSpace
import numpy as np
import tensorflow as tf

class ParameterOptimisation:
    def __init__(self):
        rospy.init_node('ParameterOptimisation node initialiser') 
        try: 
            self.pdf = PDF()
            self.env = Enviornment()
            self.search_space = SearchSpace()
            
            # Resume related code
            self.resume = True
            self.resume_data = None
            self.resume_settings = rospy.get_param('resume')
            self.path_to_saved_model = self.resume_settings['path_to_saved_model']
            self.path_to_all_models = self.resume_settings['path_to_all_models']
            self.path_to_data_json = self.resume_settings['path_to_data_json']
            exploration = None

            # Loading the json file for resume operation
            if os.path.isfile(self.path_to_data_json):
                with open(self.path_to_data_json) as json_file:
                    self.resume_data = json.load(json_file)
                    exploration = self.resume_data['exploration']
                    if 'episode' not in self.resume_data.keys():
                        self.resume = False
                    else:
                        print("WE ARE RESUMING!!!!!!")
            else:
                self.resume = False
                print("WE ARE NOT RESUMING, BUT WE ARE STARTING A BRAND NEW MODEL!!!!!!")

            
            # Setting the parameter for resume operation
            if self.resume:
                # Getting pedestrian speed and accelaration from saved data
                self.pdf.speed_ped_dist = self.resume_data['ped_speed_dist']
                self.pdf.acceleration_ped_dist = self.resume_data['ped_accel_dist']
                self.pdf.weather_dist = self.resume_data['weather_dist']

            # Add paprameters to search space
            self.add_params_to_search_space()
            self.scenario = Scenario(client = self.env.client, world = self.env.world, pdf = self.pdf, search_space = self.search_space)
            self.reward_settings = rospy.get_param('rewards')
            
            # Setting the dictionary to store the plotting data
            self.plotting_data = {
                'exploration': [], 
                'loss_per_episode': [], 
                'episode': None, 'reward': [], 
                'no_explored_step_batch': [], 
                'rss_data_per_episode': [],
                'scenario_data_per_episode': [],
                'action_per_episode': []
            }
            self.start_time_episode = time.time()
            print("Start time: {}".format(self.start_time_episode))

            # Controller is a Policy Gradient LSTM Network 
            self.controller = Controller(search_space = self.search_space, resume = self.resume, model_name = self.path_to_saved_model, exploration = exploration)
            self.process()
        except:
            e = sys.exc_info()[0]
            print(e)

        rospy.on_shutdown(self.shutdown)   
        rospy.spin()

    def add_params_to_search_space(self):
        self.search_space.add_parameters(name="ego_long_pos", values=[1, 2, 3, 4, 5, 6, 7, 8, 9, 10])
        self.search_space.add_parameters(name="ped_accel", values=self.pdf.acceleration_ped_dist)
        self.search_space.add_parameters(name="ped_speed", values=self.pdf.speed_ped_dist)

        # Do not change the below values as pedestarian spawning error occur in other positions. 
        self.search_space.add_parameters(name="ped_long_pos", values=[3, 3.5, 4, 4.5])

        # Weather presets: ['Clear Noon', 'Clear Sunset','Cloudy Noon', 'Cloudy Sunset', 'Default', 'Hard Rain Noon', 
        # 'Hard Rain Sunset', 'Mid Rain Sunset', 'Mid Rainy Noon', 'Soft Rain Noon', 'Soft Rain Sunset', 
        # 'Wet Cloudy Noon','Wet Cloudy Sunset','Wet Noon','Wet Sunset']
        self.search_space.add_parameters(name="weather", values=self.pdf.weather_dist)
        self.search_space.add_parameters(name="ped_speed_change", values=[0.50, -0.50, 0.75, -0.75])
        self.search_space.add_parameters(name="ped_timesteps", values=[20, 30, 40, 50, 60])

    def process(self):

        self.start_time_scenario = time.time()
        print("Start time: {}".format(self.start_time_scenario))
        con_settings = rospy.get_param('nas_controller')
        pg_settings = rospy.get_param('policy_gradient')
        plot_settings = rospy.get_param('plot')

        # Initilizing the values
        range_start = 0
        data_file_elapsed_time = 0
        exploration = pg_settings['exploration']

        # Setting the parameter for resume operation
        if self.resume:
            range_start = int(self.resume_data['episode'])+1
            data_file_elapsed_time = self.resume_data['elapsed_time']
            exploration = self.resume_data['exploration']

        print("Sample from pedestrian speed normal distribution: {}".format(self.pdf.speed_ped_dist))
        print("Sample from pedestrian acceleration uniform distribution: {}".format(self.pdf.acceleration_ped_dist))
        print("Sample from weather uniform distribution: {}".format(self.pdf.weather_dist))
        
        # Getting the initial random state/action
        state = self.search_space.get_random_parameter()

        # Reshaping state/action to the imput shape of the NN
        state = state.reshape(con_settings['batch_size'], con_settings['time_steps'], self.search_space.size)
        state = tf.cast(state, tf.float32)
        print("Initial state: {}".format(state))
        
        for episode in range(range_start, con_settings['epochs']): 
            try:
                # Logic for brake the looping abruptly.
                stop_looping = rospy.get_param('stop_looping')
                if stop_looping:
                    break
                
                start_time = time.time()
                print("\n\n--------------------------Episode: {}---------------------------".format(episode))
                self.env.set_all_trafficlights_to_green()
                action = self.controller.get_action(state) 
                print("Selected action: {}".format(action))
                skip = False
                reward = 0
                # Run the action

                #self.scenario.generate_ped_trajectory(episode = episode)
                reward, skip, rss_plot_data, scenario_plot_data = self.scenario.generate(actions = action, episode = episode)
                print("Reward is: {}".format(reward))
                self.env.world.tick()
                self.scenario.reset()
                self.env.world.tick()

                # Checking skip set to True 
                if skip is not True:
                    
                    print("Scenario completed and rest of the processing started")

                    # In our sample action is equal state
                    state = action.reshape(con_settings['batch_size'], con_settings['time_steps'], self.search_space.size)

                    # Casting to float
                    state = tf.cast(state, tf.float32)
                    
                    # Save the data
                    self.controller.remember(state, reward)

                else:
                    print("We are skipping this episode as something goes wrong in the scenario generation")

                 # Exploration mechanism
                if episode%pg_settings['update_exploration'] == 0:
                    if episode >= pg_settings['exploration_hard_stop']:
                        self.controller.exploration = 0
                        exploration = 0
                    else:
                        exploration = self.controller.exploration_decay_progress()

                # Save plotting data
                self.plotting_data['exploration'].append(exploration)
                self.plotting_data['episode'] = episode
                self.plotting_data['reward'].append(reward)
                self.plotting_data['rss_data_per_episode'].append(rss_plot_data)
                self.plotting_data['scenario_data_per_episode'].append(scenario_plot_data)

                #print("{}_{}_{}_{}_{}".format(str(action[0][0].item()), str(action[0][1].item()), str(action[0][2].item()), str(action[0][3].item()), str(action[0][4].item())))
                self.plotting_data['action_per_episode'].append("{}_{}_{}_{}_{}".format(str(action[0][0].item()), str(action[0][1].item()), str(action[0][2].item()), str(action[0][3].item()), str(action[0][4].item())))
                  
                if episode%pg_settings['update_every'] == 0 and episode != 0:

                    # Update the policy parameter \theta
                    self.controller.update_policy()

                    # Save plotting data
                    self.plotting_data['loss_per_episode'].append(float(self.controller.total_pg_loss))

                    # This determines no. of explored actions in a batch 
                    self.plotting_data['no_explored_step_batch'].append(self.controller.no_explored_step_batch)

                    file_location = plot_settings['path_to_plot_data'].format(episode)
                    with open(file_location, 'w') as file_handle:
                        json.dump(self.plotting_data, file_handle)
                    self.plotting_data = {
                        'exploration': [], 
                        'loss_per_episode': [], 
                        'episode': None, 
                        'reward': [], 
                        'no_explored_step_batch': [], 
                        'rss_data_per_episode': [],
                        'scenario_data_per_episode': [],
                        'action_per_episode': []
                    }

                    # Reset the variables of the controller for next batch
                    self.controller.reset()

                # Display statistics of each episode
                end_time = time.time()
                episode_time =  end_time - start_time 
                total_time_spend_sofar = (start_time-self.start_time_episode)+data_file_elapsed_time
                print("\n\nTotal elapsed time: {} hr(s)".format(round((total_time_spend_sofar/60)/60),2))
                print("Episode time: {} s".format(episode_time))
                print("Current exploration value: {}".format(float(exploration)))
                
                # Save the model and resume data      
                if episode%self.resume_settings['save_model'] == 0:
                    
                    # Save the model
                    self.controller.save_model(self.path_to_saved_model, self.path_to_all_models.format(episode))
                    
                    # Save necessary data to resume 
                    data = {
                        'model': self.path_to_saved_model,
                        'episode': episode,
                        'elapsed_time': total_time_spend_sofar,
                        'exploration' : exploration,
                        'ped_speed_dist': self.pdf.speed_ped_dist, 
                        'ped_accel_dist': self.pdf.acceleration_ped_dist,
                        'weather_dist': self.pdf.weather_dist
                    }

                    # Save the json file
                    with open(self.path_to_data_json, 'w') as outfile:
                        json.dump(data, outfile)
                
            except:
                e = sys.exc_info()[0]
                print(e)
                print("Exception occured and skipping this episode")
                print("Resetting...")
                self.scenario.reset()                
                pass
        

    def shutdown(self):
        self.scenario.destroy()
        

if __name__ == '__main__':
    try:
        ParameterOptimisation()
    except rospy.ROSInterruptException:
	    rospy.logerr('Could not start ParameterOptimisation node.')