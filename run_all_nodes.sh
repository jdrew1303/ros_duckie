#!/bin/bash

set -e -x

source /home/ros_bot/docker_setup.sh

source /home/ros_bot/devel/setup.bash

source devel/setup.sh

roscore &

# give roscore a few seconds to startup
sleep 10

# start up our services in the background (easier than a launch file)
rosrun cv_lane_follower lane_follower_node & 
rosrun differential differential_drive_node &
sleep 5

# now uses https://github.com/UbiquityRobotics/raspicam_node instead of custom node
roslaunch raspicam_node camerav2_1280x960.launch height:=100 width:=160 framerate:=20 enable_imv:=true private_topics:=true &


while sleep 60; do
  rostopic list
done
