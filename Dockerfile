FROM arm32v7/ros:kinetic-ros-base-xenial

# tag our build with git information
LABEL git_commit=$GIT_COMMIT
LABEL git_commit_author=$GIT_COMMIT_AUTHOR

# 
ENV INITSYSTEM off
ENV QEMU_EXECVE 1
# setup environment
ENV TERM "xterm"
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8
ENV ROS_DISTRO kinetic

# install ros bits and pieces
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys 421C365BD9FF1F717815A3895523BAEEB01FA116 && \
    echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list && \
    apt-get update && apt-get install -y \
    software-properties-common \
    ros-kinetic-perception \
    ros-kinetic-tf-conversions \
    ros-kinetic-joy \
    ros-kinetic-ackermann-msgs \
    python-pip \
    python-smbus \
    && apt-get update

# add the raspberry pi camera node from ubiquity robotics
RUN add-apt-repository "deb http://mirrordirector.raspbian.org/raspbian/ stretch main contrib non-free rpi" && \
    add-apt-repository "deb http://archive.raspberrypi.org/debian/ stretch main ui" && \
    curl -so - http://archive.raspberrypi.org/debian/raspberrypi.gpg.key | apt-key add - && \
    apt-get update && \
    apt-get install libraspberrypi0

RUN curl 'https://packages.ubiquityrobotics.com/ubuntu/ubiquity/pool/main/r/ros-kinetic-raspicam-node/ros-kinetic-raspicam-node_0.4.0-2xenial_armhf.deb' --output "ros-kinetic-raspicam-node_0.4.0-2xenial_armhf.deb"
RUN apt install ./ros-kinetic-raspicam-node_0.4.0-2xenial_armhf.deb && rm -f ./ros-kinetic-raspicam-node_0.4.0-2xenial_armhf.deb
RUN apt-get -f install

RUN mkdir /home/ros_bot/
COPY . /home/ros_bot

ENV READTHEDOCS True
RUN pip install -r /home/ros_bot/requirements.txt

ENV ROS_LANG_DISABLE=gennodejs:geneus:genlisp
RUN /bin/bash -c "cd /home/ros_bot/ && source /opt/ros/kinetic/setup.bash && catkin_make -j" && \
    echo "source /home/ros_bot/docker_setup.sh" >> ~/.bashrc && \
    /bin/bash -c "source /home/ros_bot/docker_setup.sh"

WORKDIR /home/ros_bot

ENV LD_LIBRARY_PATH /opt/vc/lib

RUN ["chmod", "a+x", "./run_all_nodes.sh"]
CMD ["./run_all_nodes.sh" ]
