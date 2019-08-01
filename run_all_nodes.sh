#!/bin/bash

set -e -x

source /home/duckiebot-interface/docker_setup.sh

source /home/duckiebot-interface/catkin_ws/devel/setup.bash

roscore &

source devel/setup.sh && rosrun camera camera_node &

rostopic list