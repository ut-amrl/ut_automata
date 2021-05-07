#!/bin/bash

# Source environment variables
source /opt/ros/melodic/setup.bash

# Adding the paths to required packages to ROS_PACKAGE_PATH
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/ut_automata
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/amrl_libraries/amrl_maps
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/amrl_libraries/amrl_msgs
export DISPLAY=:0 

rosrun ut_automata gui