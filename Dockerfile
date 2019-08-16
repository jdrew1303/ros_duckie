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

# add extra deb repos to the system
RUN sudo sh -c 'echo "deb https://packages.ubiquityrobotics.com/ubuntu/ubiquity xenial main" > /etc/apt/sources.list.d/ubiquity-latest.list'

# setup keys
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys 421C365BD9FF1F717815A3895523BAEEB01FA116
RUN apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key C3032ED8

# setup sources.list
RUN echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list

RUN apt-get update && apt-get install -y \
    ros-kinetic-perception \
    ros-kinetic-tf-conversions \
    ros-kinetic-joy \
    ros-kinetic-raspicam-node \
    ros-kinetic-ackermann-msgs \
    python-pip \
    python-smbus

RUN mkdir /home/ros_bot/
COPY . /home/ros_bot

ENV READTHEDOCS True
RUN pip install -r /home/ros_bot/requirements.txt

ENV ROS_LANG_DISABLE=gennodejs:geneus:genlisp
RUN /bin/bash -c "cd /home/ros_bot/ && source /opt/ros/kinetic/setup.bash && catkin_make -j"

RUN echo "source /home/ros_bot/docker_setup.sh" >> ~/.bashrc
RUN /bin/bash -c "source /home/ros_bot/docker_setup.sh"

WORKDIR /home/ros_bot

ENV LD_LIBRARY_PATH /opt/vc/lib

RUN ["chmod", "a+x", "./run_all_nodes.sh"]
CMD ["./run_all_nodes.sh" ]
