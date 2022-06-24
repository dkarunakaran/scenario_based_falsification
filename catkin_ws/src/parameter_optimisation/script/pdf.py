#!/usr/bin/env python

from __future__ import division
import rospy
import carla
import time
import json
import os
import sys
import numpy as np
import random

class PDF:

    def __init__(self):
        self.parameters = rospy.get_param('parameter_range')
        self.create_speed_ped_dist()
        self.create_accel_ped_dist()
        self.create_weather_dist()


    def create_speed_ped_dist(self):
        
        # Speed normally distribution 
        mean = self.parameters['pedestrian']['trajectory']['speed_distribution']['mean']
        sd = self.parameters['pedestrian']['trajectory']['speed_distribution']['SD']
        size = self.parameters['pedestrian']['trajectory']['speed_distribution']['size'][0]
        self.speed_ped_dist = np.random.normal(mean, sd, size)
        self.speed_ped_dist = [round(num, 3) for num in self.speed_ped_dist]

    def create_accel_ped_dist(self):

        # Acceleration uniform distribution
        low = self.parameters['pedestrian']['trajectory']['acceleration_distribution']['low']
        high = self.parameters['pedestrian']['trajectory']['acceleration_distribution']['high']
        size = self.parameters['pedestrian']['trajectory']['acceleration_distribution']['size'][0]
        self.acceleration_ped_dist = list(np.random.uniform(low, high, size))
        self.acceleration_ped_dist = [round(num, 3) for num in self.acceleration_ped_dist]

    def min_max(self, observation):
        max = observation[0]
        min = observation[1]
        if observation[1] >= max:
            max = observation[1]
            min = observation[0]

        return (min, max)


    def observation(self, typed = 'speed_ped', index = None):

        if typed == 'speed_ped':
            return self.min_max(self.speed_ped_dist[index])
        elif typed == 'accel_ped':
            return self.acceleration_ped_dist[index]

    def generate_trajectory(self, speed = None, accel = None, ped_speed_change = None, ped_timesteps = None):
        '''
        This function generate a pedestrian trajctory of pedestrians upto 100 steps.
        Logic here is upto cetain timesteps(eg. 5) it will have a less speed compared to the maximum speed provided to the function.
        After certain timesteps, it will gradually increases the speed upto the speed+0.25, then it the speed will stay as it is.
        '''

        starting_speed = speed-(accel*self.parameters['pedestrian']['trajectory']['starting_speed_upto'])
        starting_speed = round(starting_speed, 3)
        trajectory = []
        for index in range(self.parameters['pedestrian']['trajectory']['timesteps']):
            if index >= ped_timesteps and  index <= (ped_timesteps+self.parameters['pedestrian']['trajectory']['change_speed_timesteps']): 
                trajectory.append(round(starting_speed+ped_speed_change, 3))
            else:
                if index <= self.parameters['pedestrian']['trajectory']['starting_speed_upto']:
                    trajectory.append(round(starting_speed, 3))
                else:
                    if starting_speed > (speed+self.parameters['pedestrian']['trajectory']['speed_upper_limit']):
                        trajectory.append(round(starting_speed, 3))
                    else:
                        starting_speed += accel
                        trajectory.append(round(starting_speed, 3))

        return trajectory

    def create_weather_dist(self):
        low = self.parameters['weather']['weather_distribution']['low']
        high = self.parameters['weather']['weather_distribution']['high']
        size = self.parameters['weather']['weather_distribution']['size']
        self.weather_dist = list(np.random.random_integers(low, high, size))
        #self.weather_dist = [round(num, 3) for num in self.weather_dist]

    

if __name__ == '__main__':
    PDF()
