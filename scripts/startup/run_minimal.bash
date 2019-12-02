#!/usr/bin/env bash
 
USER="amrl_user"
PATH_TO_VESC="/home/$USER/vesc"
PATH_TO_LIDAR_LAUNCH=/home/$USER/f1tenth_course/src/hokuyo/launch/hokuyo_10lx.launch

# Source environment variables
source /opt/ros/melodic/setup.bash
# Adding the paths to required packages to ROS_PACKAGE_PATH
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/home/$USER/f1tenth_course/
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/home/$USER/vesc/
export DISPLAY=:0 
  
  
  
# Run VESC driver 
roslaunch vesc vesc_driver_node.launch > /dev/null 2>&1 &

# Run motion profiler
rosrun f1tenth_course motion_profiler --input=velocity > /dev/null 2>&1 &
 
# Run the joystick driver
pushd $PATH_TO_VESC
./scripts/joystick_teleop.py > /dev/null 2>&1 &
popd

# Hokuyo Lidar
roslaunch $PATH_TO_LIDAR_LAUNCH  > /dev/null 2>&1 &

