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

# setup keys
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys 421C365BD9FF1F717815A3895523BAEEB01FA116 && \
    echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list && \
    apt-get update && apt-get install -y \
    ros-kinetic-perception \
    ros-kinetic-tf-conversions \
    ros-kinetic-joy \
    ros-kinetic-ackermann-msgs \
    python-pip \
    python-smbus

# manually install camera node (I couldnt get the repo working -_-)
ENV TEMP_DEB="$(mktemp)"
RUN wget -O "$TEMP_DEB" 'https://packages.ubiquityrobotics.com/ubuntu/ubiquity/pool/main/r/ros-kinetic-raspicam-node/ros-kinetic-raspicam-node_0.4.0-2xenial_armhf.deb'
RUN apt-get install "$TEMP_DEB" && rm -f "$TEMP_DEB"

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
