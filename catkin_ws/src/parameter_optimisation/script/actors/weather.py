#!/usr/bin/env python

from __future__ import division
import rospy
import carla
import time
import json
import os
import sys

class Weather:

    def __init__(self, weather):
        self.weather = weather
        self.set_weather()

    def set_weather(self):
        self.weather.cloudiness=0.0
        self.weather.precipitation=0.0
        self.weather.precipitation_deposits=0.0
        self.weather.wind_intensity=0.0
        self.weather.sun_azimuth_angle=0.0
        self.weather.sun_altitude_angle=0.0
        self.weather.fog_density=0.0
        self.weather.fog_distance=0.0
        self.weather.wetness=0.0
    

if __name__ == '__main__':
    sample()
