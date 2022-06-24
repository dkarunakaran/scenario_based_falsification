FROM ubuntu:18.04

# Cloning git repo
RUN apt-get update

# Setup git
RUN apt-get install -y git
RUN git config --global user.name "Dhanoop Karunakaran"
RUN git config --global user.email "dkar9051@uni.sydney.edu.au"

# IMPORTANT TO CHANGE DEPENDS ON THE CONFIG YOU HAVE: ADD <ssh private file name> /root/.ssh/id_rsa
ADD .ssh /root/.ssh
RUN touch /root/.ssh/known_hosts

# IMPORTANT TO CHANGE DEPENDS ON THE GIT REPO YOU HAVE: RUN ssh-keyscan <git repo domain> >> /root/.ssh/known_hosts
RUN ssh-keyscan gitlab.acfr.usyd.edu.au >> /root/.ssh/known_hosts
RUN apt-get update

# INSTALL OTHER NECESSARY PACKAGES
RUN apt-get install -y vim
RUN apt-get install -y wget
RUN apt-get update
RUN apt-get install -y python-pip
RUN apt-get install -y libpng16-16
RUN apt-get install -y libjpeg-turbo8
RUN apt-get install -y libtiff5
RUN apt-get update
RUN pip install tensorflow==2.0.0b0
#RUN pip install -y gym

RUN apt-get install -y curl
RUN apt-get install -y lsb-core
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
RUN curl -ssl 'http://keyserver.ubuntu.com/pks/lookup?op=get&search=0xC1CF6E31E6BADE8868B172B4F42ED6FBAB17C654' | apt-key add -
RUN apt update
ARG DEBIAN_FRONTEND=noninteractive
RUN apt install -y ros-melodic-desktop
RUN apt-get install -y python-rosdep
RUN rosdep init
RUN rosdep update
RUN echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
#RUN source ~/.bashrc
RUN apt install -y python-rosinstall python-rosinstall-generator python-wstool build-essential

# Intalling pyton-catkin
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list'
RUN wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
RUN apt-get update
RUN apt-get install -y python-catkin-tools
RUN apt-get install -y software-properties-common


### Ubuntu related permission issue ###

#-------------CODE FOR AVOIDING THE PERMISSION ISSUE ON HOST THAT WILL OCCUR IF WE EDIT THE DOCKER CREATED FILES - starts----------------

# Backing up the existing docker user base
ADD backup_docker_users.bash /
RUN ["chmod", "+x", "/backup_docker_users.bash"]
RUN /backup_docker_users.bash

# Copying the same user name password from the host system If host and docker container is using ubuntu
COPY group /etc/group 
COPY passwd /etc/passwd
COPY shadow /etc/shadow

# Copying root config files to the user we selected in the file
ADD copy_to_user.bash /
RUN ["chmod", "+x", "/copy_to_user.bash"]
RUN /copy_to_user.bash

#-------------CODE FOR AVOIDING THE PERMISSION ISSUE ON HOST THAT WILL OCCUR IF WE EDIT THE DOCKER CREATED FILES - ends----------------


RUN ssh-keyscan github.com >> /root/.ssh/known_hosts
RUN apt-get update

# Get compiled version of carla-simulator
RUN cd /opt && wget https://carla-releases.s3.eu-west-3.amazonaws.com/Linux/CARLA_0.9.9.tar.gz && mkdir carla-simulator && tar -xvf CARLA_0.9.9.tar.gz -C carla-simulator && rm -f CARLA_0.9.9.tar.gz

# Clone parameter_optimisation_rl repo and change to branch
RUN cd / && git clone git@gitlab.acfr.usyd.edu.au:dkar9051/parameter_optimisation_rl.git && cd /parameter_optimisation_rl && git checkout -b development origin/development
RUN cd /parameter_optimisation_rl/catkin_ws && catkin init 

# Build catkin
ADD build_catkin.bash /
RUN ["chmod", "+x", "/build_catkin.bash"]
RUN /build_catkin.bash

# Build catkin
ADD run_initially_on_container.bash /
RUN ["chmod", "+x", "/run_initially_on_container.bash"]

CMD ["tail", "-f", "/dev/null"]