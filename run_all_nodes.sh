#!/bin/bash

set -e -x

source /home/ros_bot/docker_setup.sh

source /home/ros_bot/devel/setup.bash

source devel/setup.sh

roscore &
sleep 5
rosrun camera camera_node &
sleep 5
rostopic list
