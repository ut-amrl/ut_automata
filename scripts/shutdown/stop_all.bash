#!/usr/bin/env bash

# Kill motion profiler
rosnode kill /motion_profiler &

# Kill Cameras and Lidar
rosnode kill /urg_node &

# Kill VESC driver and joystick
rosnode kill /vesc &
rosnode kill /joystick_teleop &
