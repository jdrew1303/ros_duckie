FROM arm32v7/ros:kinetic-ros-base-xenial

ENV INITSYSTEM off
ENV QEMU_EXECVE 1
# setup environment
ENV TERM "xterm"
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8
ENV ROS_DISTRO kinetic

COPY ./qemu/bin/ /usr/bin/

RUN [ "cross-build-start" ]

# setup keys
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys 421C365BD9FF1F717815A3895523BAEEB01FA116

# setup sources.list
RUN echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list

RUN apt-get update && apt-get install -y \
    ros-kinetic-tf-conversions \
    ros-kinetic-joy \
    python-pip \
    python-smbus

# RPi libs
ADD qemu/vc.tgz /opt/
COPY qemu/00-vmcs.conf /etc/ld.so.conf.d
RUN ldconfig


# COPY requirements.txt /requirements.txt

# ENV READTHEDOCS False
# RUN pip install -r /requirements.txt

RUN mkdir /home/ros_bot/
COPY . /home/ros_bot

ENV ROS_LANG_DISABLE=gennodejs:geneus:genlisp
RUN /bin/bash -c "cd /home/ros_bot/ && source /opt/ros/kinetic/setup.bash && catkin_make -j -C catkin_ws/"

RUN echo "source /home/ros_bot/docker_setup.sh" >> ~/.bashrc
RUN bash -c "source /home/ros_bot/docker_setup.sh"

RUN [ "cross-build-end" ]

WORKDIR /home/ros_bot

CMD [ "./run_all_nodes.sh" ]