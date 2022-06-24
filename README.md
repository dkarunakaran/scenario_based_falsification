# Parameter optimisation technique using RL and Carla

This paper proposes a Reinforcement Learning (RL) based scenario-based falsification method to search for a risky scenario in a pedestrian crossing traffic situation where a Collision Avoidance System(SUT) does not satisfy the requirement. The reward function for our RL approach is based on Intel's Responsibility Sensitive Safety(RSS), Euclidean distance, and distance to the collision.

<p float="left">
  <img src="https://github.com/chinitaberrio/scenario_based_falsification/blob/main/generated%20scenario%20videos/challenging%20scenario.gif" width="400" />
  <img src="https://github.com/chinitaberrio/scenario_based_falsification/blob/main/generated%20scenario%20videos/non_challenging.gif" width="400" /> 
</p>

## Steps to setup the containers

* Build the docker image
```
bash build_docker_image.bash
```

* Build the container
```
bash create_container.bash
```

* Create and run carla docker container(optional)
```
bash create_carla_container.bash
```

* To stop container
```
docker stop phd
```

* To start container
```
docker start phd
```

* Get into the container
```
docker exec -it phd /bin/bash
```

### Best practice
It is better to stop the container once we stop using it and resume whenever needed.


## ROS bridge installation

* Create workspace called catkin_ws
* Run below command in src folder
```
git clone https://github.com/carla-simulator/ros-bridge.git
```
* Change directory to catkin_ws
* Run below commands to install the dependencies of ROS bridge
```
rosdep update
rosdep install --from-paths src --ignore-src -r
```

* Run below command to build ros bridge in catkin_ws
```
catkin_make
```

## Avoiding permission issue on host that occured while editing the docker created files on host(OS: Ubuntu)
There are a few bash scripts added to Dockerfile to avoid the permission issue. There are number of ways we can solve this issue. The way I resolved this issue by copying the userbase of host to docker container. This is recommnded only if we are not sharing the containers.

One last step that we need to do after building the image is: changing the ownership of the shared volume to the the host's user in the container. Also, copy_to_user.bash has hardcoded my user, but if it is used in another system, need to modify the hardcoded value to the user in that system.

Reference: https://medium.com/faun/set-current-host-user-for-docker-container-4e521cef9ffc

## Troubleshooting

* If you are facing the issue "ImportError: No module named carla", the follow the instructions added in readme file of the ros-bridge repo


* If you are having facing issues like "ImportError: cannot import name CarlaTrafficLightInfo", you likely to have failed to clone the carla_msgs folder. Follow below instruction to solve this issue:
```
1) git clone https://github.com/carla-simulator/ros-carla-msgs.git in catkin_ws/src/ros-bridge
2) it will be cloned as ros-carla-msgs and change name of the folder to carla_msgs
3) catkin build
```

* Error: "terminate called after throwing an instance of 'std::bad_weak_ptr'"

Reason: Issue was due to double delete. Custom code for this project has attempted to delete the actors its created and then bridge.py also tried to delete upon the update of actors. This was causing the issue.

Solution: We had to explictly not to delete the actors that had already deleted in the custom code. bridge.py in carla_ros_bridge has been modified to do that/




