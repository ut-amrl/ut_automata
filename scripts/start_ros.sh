#!/bin/bash

# Source environment variables
source /opt/ros/melodic/setup.bash

# Adding the paths to required packages to ROS_PACKAGE_PATH
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/f1tenth_course

IPADDR="$(ip addr show wlan0 | grep -Po 'inet \K[\d.]+')"
if [[ "$IPADDR" == "10.147."* ]]; then
  echo "Setting ROS_IP to $IPADDR"
  export ROS_IP=$IPADDR
  export ROS_MASTER_URI=http://$IPADDR:11311
fi

roscore
