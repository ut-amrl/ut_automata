#!/usr/bin/env bash
 
USER="amrl_user" 
PROJECT_DIR="/home/$USER/f1tenth_course" 
# Source environment variables
source /opt/ros/melodic/setup.bash
# Adding the paths to required packages to ROS_PACKAGE_PATH
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/home/$USER/f1tenth_course
export DISPLAY=:0 
  
   
# Kill joystick node if it is already running
rosnode kill /joystick_teleop
 
# Run the joystick driver
pushd $PROJECT_DIR
./scripts/joystick_teleop.py > /dev/null 2>&1 &
popd

