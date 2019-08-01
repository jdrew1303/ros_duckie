#!/bin/bash

set -e -x

source /home/ros_bot/docker_setup.sh

source /home/ros_bot/devel/setup.bash

roscore &

source devel/setup.sh && rosrun camera camera_node &

rostopic list