#!/bin/bash

# Source environment variables
source /opt/ros/jazzy/setup.bash

if [ -f "$HOME/ut_automata_ws/install/setup.bash" ]; then
  source "$HOME/ut_automata_ws/install/setup.bash"
elif [ -f "$HOME/ut_automata/install/setup.bash" ]; then
  source "$HOME/ut_automata/install/setup.bash"
fi

export DISPLAY=:0 

IPADDR="$(ip addr show wlan0 | grep -Po 'inet \K[\d.]+')"
echo "wlan0 IP address is $IPADDR"

ros2 launch ut_automata start_car.launch.py
