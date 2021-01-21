#!/bin/bash

# Source environment variables
source /opt/ros/melodic/setup.bash

# Adding the paths to required packages to ROS_PACKAGE_PATH
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/ut_automata
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/amrl_libraries/amrl_maps
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/amrl_libraries/amrl_msgs
export DISPLAY=:0

IPADDR="$(ip addr show wlan0 | grep -Po 'inet \K[\d.]+')"
echo "wlan0 IP address is $IPADDR"


# Print the commands being executed, and exit if any command fails.
set -x -e

roscd ut_automata

# Run VESC driver 
./bin/vesc_driver > /dev/null &

# Run the joystick driver
./bin/joystick --idx 1 > /dev/null &

# Hokuyo Lidar
roslaunch launch/hokuyo_10lx.launch  > /dev/null &

# Run Websocket server.
./bin/websocket > /dev/null &

# Run the GUI
./bin/gui
