#!/bin/bash

if [ -z "$1" ]
  then
    echo "**************************************************************"
    echo "No first argument supplied, Select one of the below"
    echo "----------------------------------------------------"
    echo ""
    echo "start_server: Pull and start the carla server docker" 
    echo ""
    echo "stop_server: Stop the carla server docker" 
    echo ""
    echo "start_client: Start the client docker"
    echo ""
    echo "resume_client: Resume the already started client docker"
    echo ""
    echo "stop_client: Stop the client docker"
    echo ""
    echo "remove_untagged: Remove the untagged images"
    echo ""
    echo "build_client_i: Build the client docker image"
    echo ""
    echo "build_client_c: Build the client docker container"
    echo ""
    echo "**************************************************************"
    echo ""
    echo "And pass second argument as well: 'laptop or monolith'"
    echo ""
    exit 1
fi

if [ -z "$2" ]
  then
    echo "No second argument supplied, pass 'laptop or monolith'"
    exit 1
fi

server_docker_c=carla-sim-0-9-9
client_docker_i=paper2_i
client_docker_c=paper2_c
carla_version=carlasim/carla:0.9.9

echo "Running '$1' and '$2' options"

# Start the carla server
if [[ $1 == "start_server" ]]; then
	docker pull carlasim/carla:0.9.9
	if [[ $2 == "laptop" ]]; then
  		docker run --name=$server_docker_c -d=true --net=host --runtime=nvidia -e NVIDIA_VISIBLE_DEVICES=0 $carla_version /bin/bash CarlaUE4.sh
	fi
	
	if [[ $2 == "monolith" ]]; then
  		docker run --name=$server_docker_c -d=true --net=host --gpus 2 $carla_version /bin/bash CarlaUE4.sh
	fi
fi

# Stop the carla server
if [[ $1 == "stop_server" ]]; then
        docker stop $server_docker_c
        docker rm $server_docker_c
fi

# Start the client docker
if [[ $1 == "start_client" ]]; then
        docker start $client_docker_c
	docker exec -it $client_docker_c /bin/bash
fi

# Stop the client docker
if [[ $1 == "stop_client" ]]; then
        docker stop $client_docker_c
fi

# Resume the client docker
if [[ $1 == "resume_client" ]]; then
	docker exec -it $client_docker_c /bin/bash
fi

# Remove untagged docker images
if [[ $1 == "remove_untagged" ]]; then
	docker rmi -f $(docker images --filter "dangling=true" -q --no-trunc)
fi

# Build docker client image
if [[ $1 == "build_client_i" ]]; then

        #-------------CODE FOR AVOIDING THE PERMISSION ISSUE ON HOST THAT WILL OCCUR IF WE EDIT THE DOCKER CREATED FILES - starts----------------

        # Copying the same user name password from the host system If host and docker container is using ubuntu
        cp /etc/group .
        cp /etc/passwd .
        cp /etc/shadow .

        #-------------CODE FOR AVOIDING THE PERMISSION ISSUE ON HOST THAT WILL OCCUR IF WE EDIT THE DOCKER CREATED FILES - ends----------------

        # IMPORTANT TO CHANGE DEPENDS ON THE CONFIG YOU HAVE: cp 'ssh_private_file_location' .
        cp -r ~/.ssh .

        # Build the image
        docker build -t $client_docker_i .

        #-------------CODE FOR AVOIDING THE PERMISSION ISSUE ON HOST THAT WILL OCCUR IF WE EDIT THE DOCKER CREATED FILES - starts----------------
        rm -rf .ssh
        rm -f group
        rm -f passwd
        rm -f shadow

        #-------------CODE FOR AVOIDING THE PERMISSION ISSUE ON HOST THAT WILL OCCUR IF WE EDIT THE DOCKER CREATED FILES - ends----------------
        	
fi

# Build docker client container
if [[ $1 == "build_client_c" ]]; then
	if [[ $2 == "laptop" ]]; then
  		docker run --net=host -d -v=/home/dhanoop/Documents/acfr/active/carla/carla_ros/parameter_optimisation_rl:/parameter_optimisation_rl --name $client_docker_c $client_docker_i
	fi

	if [[ $2 == "monolith" ]]; then
  		docker run --net=host -d -v=/home/dhanoop/parameter_optimisation_rl:/parameter_optimisation_rl --name $client_docker_c $client_docker_i
	fi
fi



