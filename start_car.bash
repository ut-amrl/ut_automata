#!/usr/bin/env bash


USER="amrl_user"
PROJECT_DIR="/home/$USER/f1tenth_course"
PATH_TO_LIDAR_LAUNCH=/home/$USER/f1tenth_course/src/hokuyo/launch/hokuyo_10lx.launch

# Source environment variables
source /opt/ros/melodic/setup.bash
# Adding the paths to required packages to ROS_PACKAGE_PATH
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/home/$USER/f1tenth_course
export DISPLAY=:0 

# Print the commands being executed, and exit if any command fails.
set -x -e

# Run VESC driver 
$PROJECT_DIR/bin/vesc_driver > /dev/null &

# Run the joystick driver
$PROJECT_DIR/bin/joystick --idx 1 > /dev/null &

# Hokuyo Lidar
roslaunch $PATH_TO_LIDAR_LAUNCH  > /dev/null &

