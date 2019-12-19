#!/usr/bin/env bash

USER="amrl_user"
PATH_TO_LIDAR_LAUNCH=/home/$USER/f1tenth_course/src/hokuyo/launch/hokuyo_10lx.launch

# Source environment variables
source /opt/ros/melodic/setup.bash
# Adding the paths to required packages to ROS_PACKAGE_PATH
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/home/$USER/f1tenth_course/
export DISPLAY=:0


# Hokuyo Lidar
roslaunch $PATH_TO_LIDAR_LAUNCH  > /dev/null &
