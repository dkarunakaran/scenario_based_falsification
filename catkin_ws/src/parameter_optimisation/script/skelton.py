#!/usr/bin/env python

from __future__ import division
import rospy
import carla
import time
import json
import os
import pkg_resources
from distutils.version import LooseVersion
from actors.ego_vehicle import EgoVehicle
import sys

class Sample:

    def __init__(self):
        print("hello")

    def sample_method(self):
        print("hello")
    

if __name__ == '__main__':
    Sample()
