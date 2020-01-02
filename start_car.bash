#!/bin/bash

# Source environment variables
source /opt/ros/melodic/setup.bash

# Adding the paths to required packages to ROS_PACKAGE_PATH
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/f1tenth_course
export DISPLAY=:0 

IPADDR="$(ip addr show wlan0 | grep -Po 'inet \K[\d.]+')"
echo "wlan0 IP address is $IPADDR"
if [[ "$IPADDR" == "10.147."* ]]; then
  echo "Setting ROS_IP to $IPADDR"
  export ROS_IP=$IPADDR
  export ROS_MASTER_URI=http://$IPADDR:11311
fi

# Print the commands being executed, and exit if any command fails.
set -x -e

# Run VESC driver 
./bin/vesc_driver > /dev/null &

# Run the joystick driver
./bin/joystick --idx 1 > /dev/null &

# Hokuyo Lidar
roslaunch src/hokuyo/launch/hokuyo_10lx.launch  > /dev/null &

# ROS Websocket bridge.
roslaunch rosbridge_server rosbridge_websocket.launch > /dev/null &

