#!/usr/bin/env python

from __future__ import division
import rospy
import carla
import time
import json
import os
from actors.ego_vehicle import EgoVehicle
from actors.walker import Walker
import sys
import random
import numpy as np
from pdf import PDF
import math 
import matplotlib
import re

# For the error: Exception ignored in: <bound method Image.del of <tkinter.PhotoImage object at 0x7f1b5f86a710>> Traceback (most recent call last):
matplotlib.use('Agg')
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import pyplot as plt
import tensorflow as tf
from rss import RSS

class Scenario:

    def __init__(self, client = None, world = None, pdf = None, search_space = None):
        self.client = client
        self.world = world
        self.pdf = pdf
        self.spawn_points = self.world.get_map().get_spawn_points()
        self.actor_list = []
        self.walker_list = []
        self.walker_controller_list = []
        self.parameters = rospy.get_param('parameter_range')
        self.scenario_settings = rospy.get_param('scenario')
        self.reward_settings = rospy.get_param('rewards')
        self.search_space = search_space
        self.rss = RSS()
        self.weather_preset = self.find_weather_presets()

    '''
    action = [[Ego vehicle position, no. of pedestrian, pedestrian postion, pedestrian longitudinal position]]
    '''
    def generate(self, actions = None, episode = None):

        print("Generating the scenario from: {}".format(actions))

        self.collision_flag = False

        # Setting the weather
        # weather - actions[0][3]
        weather_index = int(actions[0][3].item())
        preset = self.weather_preset[weather_index]
        self.world.set_weather(preset[0])

        # Ego vehicle position
        # action[0][0]
        long_pos = actions[0][0]
        ego_long_pos = long_pos.item()

        # Creation of Ego vehicle
        self.ego_vehicle = EgoVehicle(client = self.client, world = self.world, spawn_points = self.spawn_points, long_pos = long_pos.item())
        self.ego_vehicle.collision_sensor.listen(lambda event: self.ego_collision(event))
        self.actor_list.append(self.ego_vehicle)
        self.world.tick()

        # ped_accel - actions[0][1]
        # ped_speed - actions[0][2]
        # pedestrian longitudinal position - actions[0][3]
        no_of_ped = 1
        ped_accel = actions[0][1].item()
        ped_speed = actions[0][2].item()
        ped_long_pos = actions[0][3].item()
        ped_speed_change = actions[0][5].item()
        ped_timesteps = actions[0][6].item()
        lat_count = 0
        long_count = 1

        #Generate pedestrian trajectory
        ped_trajectory = self.pdf.generate_trajectory(speed = ped_speed, accel = ped_accel, ped_speed_change = ped_speed_change, ped_timesteps = ped_timesteps)

        # We assume that there is only one ped for the experiment
        self.walker1 = None

        # Creation of pedestrians
        for ped in range(0, int(no_of_ped)): 
            #long_location = self.search_space.ped_long_action_to_long_pos(ped_long_pos)+long_count
            long_location = ped_long_pos+long_count
            lat_location = lat_count
            key = "walker_{}".format(ped)
            walker = Walker(role_name = key, client = self.client, world = self.world, spawn_points = self.spawn_points, long_location = long_location, lat_location = lat_location)
            
            # We assume that there is only one ped for the experiment
            self.walker1 = walker 

            self.actor_list.append(walker)
            self.walker_list.append(walker)
            lat_count += 1
            long_count += 1
            self.world.tick()
            
        ego_init_loc = None
        ego_long_loc = None
        reward = 0
        eucld_dist = 200
        ped_speed_count = 0

        # There is an error where sometimes pedestrians are not spwaning to the simulation
        # So we need to kip such episodes
        skip = True
        for walker in self.walker_list:
            if walker.player and walker.player.is_alive:
                skip = False

        # If the RSS safe distance is less than actual distance, we need add some value to the list to compute the reward.
        rss_failure = 0 
        ttc_failure = 0 
        timesteps = 0
        timesteps_active_total = 0
        timestep_failure = []
        rss_plot_data = None
        scenario_plot_data = {
            'speed_ped': [],
            'speed_ego': [],
            'timesteps': None,
            'timestep_failure': None,
            'euclidean_distance': []
        }

        if skip is not True:
            start_time = time.time()
            while True:

                # Controlling the vehcile
                #self.ego_vehicle.player.apply_control(carla.VehicleControl(throttle=0.5, brake=0.0))

                # At the moment, we are testing the carla's emergency braking system setup in the auto pilot.
                self.ego_vehicle.player.set_autopilot(True)

                # Calculation for distance travelled by ego vehicle.
                if ego_init_loc == None:
                    ego_init_loc = self.ego_vehicle.get_longitudinal_location()
                    ego_dist_travelled = 0
                else:
                    ego_long_loc = self.ego_vehicle.get_longitudinal_location()
                    ego_dist_travelled = ego_long_loc - ego_init_loc

                # To make sure vehicle has started moving
                if ego_dist_travelled > 0:

                    timesteps += 1
                    loc_ego = self.ego_vehicle.player.get_location()
                    eucld_dist = None 
                    
                    ped_speed_selected = ped_trajectory[ped_speed_count]
                    ped_speed_count += 1

                    # Controlling the pedestrians
                    for walker in self.walker_list:
                        if walker.player and walker.player.is_alive:

                            # --------For Temporary reward function - Starts ------
                            # Calculating the euclidean distance between pedestrians and ego 
                            loc_a1 = walker.player.get_location()
                            actual_dist = loc_ego.distance(loc_a1)
                            if eucld_dist is None:
                                eucld_dist = actual_dist
                            elif actual_dist < eucld_dist: 
                                eucld_dist = actual_dist
                            # --------For Temporary reward function - ends ------
                                                     
                            control = walker.player.get_control()

                            # At the moment, we are setting constant velocity
                            control.speed = ped_speed_selected
                            walker.player.apply_control(control)
                
                    # This command is needed if the simulation in synchronous mode.
                    self.world.tick()
                    vel_ego = self.ego_vehicle.player.get_velocity()
                    if vel_ego > 0:
                        
                        timesteps_active_total += 1

                        # -------------------------------CALCULATE RSS STARTS------------------------------------
                        # S. Shalev-Shwartz, S. Shammah, and A. Shashua, On a formal model of safe and scalable self-driving cars, CoRR, vol. abs/1708.06374,
                        #2017. [Online]. Available: http://arxiv.org/abs/1708.06374

                        # RSS distance
                        
                        rss_dist = self.rss.calculate_rss_safe_dist(abs(vel_ego.y), 0)

                        # Actual distance
                        loc_ego = self.ego_vehicle.player.get_location()
                        loc_a1 = self.walker1.player.get_location()
                        actual_dist = loc_ego.distance(loc_a1)
                        #print(".........location: {}".format(loc_a1))

                        #print("RSS distance: {} and Actual distance: {}".format(rss_dist, actual_dist))

                        # This means vehicle is at non safe longitudinal position and proper response of the vehicle is to brake
                        if rss_dist > actual_dist:                 
                            #rss_failure += 1
                            timestep_failure.append(1)
                        else:
                            timestep_failure.append(0)
                        

                        # -------------------------------CALCULATE RSS ENDS------------------------------------
                        '''
                        # -------------------------------CALCULATE TTC STARTS------------------------------------
                        # Cafiso, S., Garcia, A.G., Cavarra, R. and Rojas, M.R., 2011, September. Crosswalk safety evaluation using a pedestrian risk index as traffic conflict measure. 
                        # In Proceedings of the 3rd International Conference on Road safety and Simulation (pp. 1-15). 
                        # Paper link: http://onlinepubs.trb.org/onlinepubs/conferences/2011/RSS/2/Cafiso,S.pdf

                        # Pedestrian crossing: 
                        # x = 25 to 37
                        # Y = -177 
                        # Lane:
                        # Each lane is 3 meters wide, 1st lane: 25 to 28, 2nd lane: 28 to 31, 3rd lane: 31 to 34, 4th lane: 34 to 37
                        # Time to Collision of the vehicle, obtained from following equation: TTC_vi = Di/Vi 
                        # where Di is the longitudinal distance between the vehicle and the crossing at the instant i and Vi is the vehicle speed at the instant i.
                        ttc_vehicle = 0
                        if abs(vel_ego.y) > 0 and abs(vel_ego.y) < 50:
                            crooswalk_y = -177 
                            dist_long_ego_cw = abs(crooswalk_y) - abs(loc_ego.y)
                            ttc_vehicle = dist_long_ego_cw/abs(vel_ego.y)
                            
                        # Time to Collision of pedestrian is carried out in order to establish if a pedestrian can
                        # arrive at the conflict area in time to collide with vehicle: TTC_pi = (Dxi(v) - Dxi(p))/V_p
                        # Dxi(v) = lateral vehicle distance at instant i.
                        # Dxi(p) =  pedestrian position on crossing at instant i. 
                        # V_p =  pedestrian speed.
                        ttc_ped = 0
                        vel_ped = self.walker1.player.get_velocity()
                        if abs(vel_ped.x) > 0:
                            dist_lateral_vehicle = 6 #m 2 lanes and vehicle is at the the centre of the third lane.

                            # ASSUMPTION: we only consider one pedestrian moving from the right of the ego vehicle. 
                            # If we need to change  direction, below calculation need to be changed 
                            # Conflict area is defined as 3 meters of the the third lane, so 31
                            conflict_area_start = 31
                            dist_lateral_ped = conflict_area_start - abs(loc_a1.x)
                            ttc_ped = (dist_lateral_vehicle - dist_lateral_ped)/abs(vel_ped.x)

                        # When ttc_vehicle>ttc_ped, the pedestrian is exposed to conflict with vehicle
                        # when ttc_vehicle<ttc_ped, the pedestrian reaches the conflict area only after the vehicle has passed(passing phase). 
                        if (ttc_vehicle > ttc_ped) and (self.scenario_settings['scenario_end']['ego_distance']-ego_dist_travelled) <= 10:
                            ttc_failure += 1
                        # -------------------------------CALCULATE TTC ENDS------------------------------------
                        '''

                    vel_ped = self.walker1.player.get_velocity()
                    scenario_plot_data['speed_ped'].append(abs(vel_ped.x))
                    scenario_plot_data['speed_ego'].append(abs(vel_ego.y))
                    scenario_plot_data['timesteps'] = timesteps
                    scenario_plot_data['timestep_failure'] = timestep_failure
                    scenario_plot_data['euclidean_distance'].append(eucld_dist)
                    
                    
                else:
                    # This command is needed if the simulation in synchronous mode.
                    self.world.tick()
                
                end_time = time.time()
                episode_time =  end_time - start_time 

                #print("timesteps: {}".format(timesteps))
                #print("timesteps_active_total: {}".format(timesteps_active_total))
                #print("timesteps_failure: {}".format(timestep_failure))

                # This is the criteria for end of scenario.
                if self.collision_flag is True or episode_time > self.scenario_settings['scenario_end']['max_time'] or ego_dist_travelled > (self.scenario_settings['scenario_end']['ego_distance']+ego_long_pos):

                    # Reward calculation:
                    # - Range: we need to reward the action that can generate the less euclidean distance because it can be more prone to failure when we have RSS
                    # - RSS: We need to reward the action that can geenerate more failure timsteps as per the RSS
                    # - Time To Collosion(TTC):
                    
                    # -------------------------------RANGE BASED REWARDS STARTS------------------------------------
                    
                    # min = 0
                    # max = 10
                    # y = (x - min) / (max - min), normalise between 0 and 1.
                    normalised = self.normalise(eucld_dist, minimum=self.reward_settings['normalise_range']['min'], maximum=self.reward_settings['normalise_range']['max'])

                    # Reverse the value such a way that the higher the euclead distance, then lower the reward and vice versa.
                    norm_reward = 1 - normalised

                    # Using this formula: https://stats.stackexchange.com/questions/178626/how-to-normalize-data-between-1-and-1
                    # Actual range is : [0, 1] - [min, max]
                    # New range is: [-0.1, 0.1] - [a,b]
                    # Equation: (b-a)*((x-min)/(max-min))+a
                    reward =  self.normalise_to_x(norm_reward, 0, 1, self.reward_settings['reward_range']['min'], self.reward_settings['reward_range']['max'])
                    
                    print("Euclidean distance: {}".format(eucld_dist))
                    print("Range reward: {}".format(reward))
                    # -------------------------------RANGE BASED REWARDS ENDS------------------------------------
                    
                    # -------------------------------RSS BASED REWARDS STARTS------------------------------------
                    
                    check = 0
                    for item in timestep_failure:
                        if item == 1:
                            rss_failure += 1
                        '''if item == 1:
                            check +=1
                        else:
                            check = 0
                        if check == self.reward_settings['reward_rss_threshold']:
                            rss_failure += 1
                            check = 0'''

                                        
                    #normalised = self.normalise(rss_failure, minimum=self.reward_settings['normalise_rss_range']['min'], maximum=self.reward_settings['normalise_rss_range']['max'])
                    normalised = self.normalise(rss_failure, minimum=self.reward_settings['normalise_rss_range']['min'], maximum=timesteps)
                    reward_rss = self.normalise_to_x(normalised, 0, 1, self.reward_settings['reward_rss_range']['min'], self.reward_settings['reward_rss_range']['max'])
                    print("RSS is failing in {} timsteps out of {} and reward_rss: {}".format(rss_failure, timesteps, reward_rss))
                    rss_plot_data = "{}_{}".format(rss_failure, timesteps)
                    reward += reward_rss
                    # -------------------------------RSS BASED REWARDS ENDS------------------------------------
                    
                    '''
                    # -------------------------------TTC BASED REWARDS STARTS------------------------------------
                    normalised = self.normalise(ttc_failure, minimum=self.reward_settings['normalise_ttc_range']['min'], maximum=self.reward_settings['normalise_ttc_range']['max'])
                    reward_ttc = self.normalise_to_x(normalised, 0, 1, self.reward_settings['reward_ttc_range']['min'], self.reward_settings['reward_ttc_range']['max'])
                    print("TTC_vel > TTC_ped in {} timsteps out of {} and reward_ttc: {}".format(ttc_failure, timesteps, reward_ttc))
                    reward += reward_ttc
                    # -------------------------------TTC BASED REWARDS ENDS------------------------------------
                    '''

                    if self.collision_flag:
                        reward += self.reward_settings['collision']
                        
                    
                    # There was runtime error due to autopilot is still running. We need to stop it before breaking the loop.
                    self.ego_vehicle.player.set_autopilot(False)

                    # This command is needed, if the simulation is in synchronous mode.
                    self.world.tick()
                    break

        return reward, skip, rss_plot_data, scenario_plot_data

    
    def ego_collision(self, event):
        self.collision_flag = True


    # Weather presets: ['Clear Noon', 'Clear Sunset','Cloudy Noon', 'Cloudy Sunset', 'Default', 'Hard Rain Noon', 
    # 'Hard Rain Sunset', 'Mid Rain Sunset', 'Mid Rainy Noon', 'Soft Rain Noon', 'Soft Rain Sunset', 
    # 'Wet Cloudy Noon','Wet Cloudy Sunset','Wet Noon','Wet Sunset']
    def find_weather_presets(self):
        rgx = re.compile('.+?(?:(?<=[a-z])(?=[A-Z])|(?<=[A-Z])(?=[A-Z][a-z])|$)')
        name = lambda x: ' '.join(m.group(0) for m in rgx.finditer(x))
        presets = [x for x in dir(carla.WeatherParameters) if re.match('[A-Z].+', x)]
        return [(getattr(carla.WeatherParameters, x), name(x)) for x in presets]

    def normalise(self, value = None, minimum = None, maximum = None):
        
        # min = 0
        # max = 10
        # y = (x - min) / (max - min), normalise between 0 and 1.
        return round(((value - minimum) / (maximum-minimum)), 2)

    def normalise_to_x(self, value = None, minimum = None, maximum = None, newrange_a = None, newrange_b = None):
        
        # Using this formula: https://stats.stackexchange.com/questions/178626/how-to-normalize-data-between-1-and-1
        # Actual range is : [0, 1] - [min, max]
        # New range is: [-0.1, 0.1] - [a,b]
        # Equation: (b-a)*((x-min)/(max-min))+a
        return ((newrange_b - newrange_a)*((value-minimum)/(maximum-minimum))+newrange_a)

    def reset(self):
        self.walker_controller_list = []
        for actor in self.actor_list:
            actor.destroy()
        self.actor_list = []
        self.walker_list = []

    def destroy(self):
        for actor in self.actor_list:
            actor.destroy()
        self.actor_list = None

if __name__ == '__main__':
    Scenario()
