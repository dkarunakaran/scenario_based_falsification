#!/usr/bin/env python

from __future__ import division
import rospy
import carla
import time
import json
import os
import sys
import pkg_resources
from distutils.version import LooseVersion
from actors.ego_vehicle import EgoVehicle
from actors.walker import Walker

class Enviornment:

    client = None
    world = None
    CARLA_VERSION = "0.9.9"
    visualize_spwan_points = False
    actor_list = [] 

    def __init__(self):

        # check CARLA version
        dist = pkg_resources.get_distribution("carla")
        if LooseVersion(dist.version) < LooseVersion(self.CARLA_VERSION):
            raise ImportError(
                "CARLA version {} or newer required. CARLA version found: {}".format(
                    self.CARLA_VERSION, dist))

        parameters = rospy.get_param('connect')
        rospy.loginfo("Trying to connect to {host}:{port}".format(
            host=parameters['host'], port=parameters['port']))
        try:
            self.client = carla.Client(
                host=parameters['host'],
                port=parameters['port'])
            self.client.set_timeout(2000)
            self.world = self.client.get_world()
            if "town" in parameters and self.world.get_map().name != parameters["town"]:
                rospy.loginfo("Loading new town: {} (previous: {})".format(
                    parameters["town"], self.world.get_map().name))
                self.world = self.client.load_world(parameters["town"])

            weather = carla.WeatherParameters(
                cloudiness=80.0,
                precipitation=30.0,
                sun_altitude_angle=70.0)

            self.world.set_weather(weather)
            self.set_all_trafficlights_to_green()

            carla_settings = rospy.get_param('carla_settings')
            settings = self.world.get_settings()
            settings.fixed_delta_seconds = carla_settings['fixed_delta_seconds']
            settings.synchronous_mode = carla_settings['synchronous_mode'] # Enables synchronous mode
            settings.synchronous_mode_wait_for_vehicle_control_command = carla_settings['vehicle_control_command'] 
            self.world.apply_settings(settings)

            # Waiting to load everything
            print("Waitinf for 3 seconds to load everything.....")
            time.sleep(3)
            print("connected.....")
            
        except:
            e = sys.exc_info()[0]
            print(e)

    def visualize_spwan_point(self):

        # Drawing points
        if self.visualize_spwan_points:
            spawn_points = self.world.get_map().get_spawn_points()
            debug = self.world.debug
            count = 0
            for waypoint in spawn_points:
                string = "Count:"+str(count)+" x:"+str(waypoint.location.x)+" y:"+str(waypoint.location.y)+" z:"+str(waypoint.location.z)+" pitch:"+str(waypoint.rotation.pitch)+" roll:"+str(waypoint.rotation.roll)+" yaw:"+str(waypoint.rotation.yaw)
                debug.draw_string(waypoint.location, str(string), draw_shadow=False,
                                                color=carla.Color(r=255, g=255, b=255), life_time=200,
                                                persistent_lines=True)
                count += 1
    
    def set_all_trafficlights_to_green(self):
        # Setting the trafiic lights to green to create the general pedestrian crossing scenario in carla.
        for tl in self.world.get_actors().filter('traffic.traffic_light*'):
            tl.set_state(carla.TrafficLightState.Green)
            tl.freeze(True)


if __name__ == '__main__':
    Enviornment()
