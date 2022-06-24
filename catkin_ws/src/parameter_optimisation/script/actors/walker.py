#!/usr/bin/env python

from __future__ import division
import rospy
import carla
import time
import json
import os
import sys
import random

class Walker:

    def __init__(self, role_name = None, client = None, world = None, spawn_points = None, long_location = None, lat_location = None):
        self.client = client
        self.world = world
        self.player = None
        parameter_config = rospy.get_param('parameter_config')
        self.initial_spawn_point = parameter_config['pedestrian']['initial_spawn_point']
        self.long_location = long_location
        self.lat_location = lat_location
        walker_type = parameter_config['pedestrian']['walker_type']
        self.spawn_points = spawn_points
        self.role_name = role_name
        self.blueprint = random.choice(self.world.get_blueprint_library().filter(walker_type))
        self.blueprint.set_attribute('role_name', "{}".format(self.role_name))
        if self.blueprint.has_attribute('is_invincible'):
            self.blueprint.set_attribute('is_invincible', 'false')

        # Set a spawn point
        self.set_spawn_point()

        # Create a walker
        self.create()
        
    
    def set_spawn_point(self):
        self.spawn_point = self.spawn_points[self.initial_spawn_point]

        # Here x is the longitudinal position
        self.spawn_point.location.x -= self.long_location
        self.spawn_point.location.y -= self.lat_location

        

    def create(self):
        print(".........{} location: {}".format(self.role_name, self.spawn_point))
        self.player = self.world.try_spawn_actor(self.blueprint, self.spawn_point)

        # Resetting the longitudinal and lateral postion
        self.spawn_point.location.x += self.long_location
        self.spawn_point.location.y += self.lat_location

    def destroy(self):
        if self.player and self.player.is_alive:
            self.player.destroy()
        self.player = None
        print("Walker removed")
    

if __name__ == '__main__':
    Walker()
