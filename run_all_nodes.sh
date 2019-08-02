#!/bin/bash

set -e -x

source /home/ros_bot/docker_setup.sh

source /home/ros_bot/devel/setup.bash

source devel/setup.sh

roscore &

# give roscore a few seconds to startup
sleep 10

# start up our services in the background (easier than a launch file)
rosrun camera camera_node &
rosrun differential differential_drive_node
