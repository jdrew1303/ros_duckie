#!/bin/bash

MY_IP=$(hostname -I | cut -d " " -f 1)
export ROS_IP=${MY_IP}
echo "Setting ROS_IP to host IP, which is $ROS_IP"
if [ ! -z "$ROS_MASTER_URI" ]; then
    echo "ROS_MASTER_URI was externally set to \"$ROS_MASTER_URI\", skipping configuration."
else # We are running on the Duckiebot, which can always reach itself
    export ROS_MASTER_URI="http://localhost:11311/"
fi