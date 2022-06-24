#!/bin/bash

echo "source /opt/ros/melodic/setup.bash" >> ~/.bash_profile
echo "export PYTHONPATH=$PYTHONPATH:/opt/carla-simulator/PythonAPI/carla/dist/carla-0.9.9-py2.7-linux-x86_64.egg" >> ~/.bashrc
echo "export PYTHONPATH=$PYTHONPATH:/opt/carla-simulator/PythonAPI/carla/dist/carla-0.9.9-py2.7-linux-x86_64.egg" >> ~/.bash_profile
source ~/.bashrc
source ~/.bash_profile

cd /parameter_optimisation_rl/catkin_ws

# Install the dependency
rosdep update
rosdep install -y --from-paths src --ignore-src --rosdistro melodic --as-root apt:false

catkin build

source devel/setup.bash
