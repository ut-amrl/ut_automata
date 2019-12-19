#!/usr/bin/env bash


USER="amrl_user"
PROJECT_DIR="/home/$USER/f1tenth_course"
PATH_TO_LIDAR_LAUNCH=/home/$USER/f1tenth_course/src/hokuyo/launch/hokuyo_10lx.launch

# Source environment variables
source /opt/ros/melodic/setup.bash
# Adding the paths to required packages to ROS_PACKAGE_PATH
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/home/$USER/f1tenth_course
export DISPLAY=:0 

IP_ADDR=$(ip addr show dev wlan0 | grep -o "inet [0-9]*.[0-9]*.[0-9]*.[0-9]*" | cut -c6-100)
echo "IP = $IP_ADDR"

# Print the commands being executed, and exit if any command fails.
set -x -e

export ROS_IP=$IP_ADDR
# Kill previous roscore to set the IP.
roscore &

sleep 6

# Run VESC driver 
roslaunch f1tenth_course vesc_driver_node.launch > /dev/null &

# Run the joystick driver
$PROJECT_DIR/bin/joystick --idx 1 > /dev/null &

# Hokuyo Lidar
roslaunch $PATH_TO_LIDAR_LAUNCH  > /dev/null &

