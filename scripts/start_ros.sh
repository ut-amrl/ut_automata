#!/bin/bash

# Source environment variables
source /opt/ros/melodic/setup.bash

# Adding the paths to required packages to ROS_PACKAGE_PATH
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/ut_automata

roscore
