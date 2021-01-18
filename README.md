# ut_automata
Repository for UT AUTOmata courses  

[![Build Status](https://travis-ci.com/ut-amrl/ut_automata.svg?branch=master)](https://travis-ci.com/ut-amrl/ut_automata)

### Overview
These instructions are tailored to the computer setup in the GDC1.310 lab. If you are setting this up on your own personal computer, you will need to modify the instructions for your own setup, including perhaps saving the setup script entries to `.bashrc` instead of `.profile`

### Update your `.profile` file
1. Add the following lines to the bottom of  `~/.profile`:
   ```
   source /opt/ros/melodic/setup.bash
   export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/ut_automata
   ```
1. If you are cloning the starter code (https://github.com/ut-amrl/cs378_starter) as well, make sure to include the path for that as well in `.profile`:
   ```
   export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/projects/cs378_starter
   ```


After adding these lines you will need to either relog into the computer or run:
`source ~/.profile`

### Dependencies
1. Run `install_dependencies.sh` to install package dependencies.
1. Clone and build [amrl_msgs](https://github.com/ut-amrl/amrl_msgs).

### Clone and Build Course Code
1. From your home directory run:
   ```
   git clone https://github.com/ut-amrl/ut_automata.git --recurse-submodule
   cd ut_automata
   make -j
   ```
2. Alternatively if you have accidently forgot the ```--recurse-submodule``` flag during cloning phase, you can use the following steps instead:
   ```
   git clone https://github.com/ut-amrl/ut_automata.git
   cd ut_automata
   git submodule init
   git submodule update
   make -j
   ```
3. To build the hardware drivers as well on the actual cars:
   ```
   make hardware
   ```

### Autostart on Actual Car Jetson Computer

To start the car driver nodes automatically when the Jetson boots up:
1. Install a symlink to the `ut_automata.service` under systemd:
   ```
   sudo ln -s (PATH_TO_REPO)/scripts/ut_automata.service /etc/systemd/system/ut_automata.service
   ```
1. Enable the service:
   ```
   sudo service enable ut_automata
   ```
1. Reboot the computer
1. When the computer reboots, it should start all driver nodes, and show the UT AUTOmata GUI.
1. You can check the status of the `ut_automata` service using:
   ```
   service ut_automata status
   ```
   Sample output:
   ```
   amrl_user@car0:~$ service ut_automata status
   ● ut_automata.service - UT AUTOmata
      Loaded: loaded (/home/amrl_user/ut_automata/ut_automata.service; enabled; vendor preset: enabled)
      Active: active (running) since Fri 2021-01-15 22:15:42 CST; 2min 0s ago
   Main PID: 5264 (bash)
      Tasks: 55 (limit: 4915)
      CGroup: /system.slice/ut_automata.service
            ├─5264 /bin/bash /home/amrl_user/ut_automata/start_car.sh
            ├─6147 /usr/bin/python /opt/ros/melodic/bin/roslaunch ut_automata start_car.launch
            ├─6740 /usr/bin/python /opt/ros/melodic/bin/rosmaster --core -p 11311 -w 3 __log:=/home/amrl_user/.ros/log/811fd94e-57b1-11eb-b3d2-521039dc703d/master.log
            ├─6843 /opt/ros/melodic/lib/rosout/rosout __name:=rosout __log:=/home/amrl_user/.ros/log/811fd94e-57b1-11eb-b3d2-521039dc703d/rosout-1.log
            ├─6846 /opt/ros/melodic/lib/urg_node/urg_node __name:=urg_node __log:=/home/amrl_user/.ros/log/811fd94e-57b1-11eb-b3d2-521039dc703d/urg_node-2.log
            ├─6863 /home/amrl_user/ut_automata/bin/websocket __name:=websocket __log:=/home/amrl_user/.ros/log/811fd94e-57b1-11eb-b3d2-521039dc703d/websocket-4.log
            ├─6867 /home/amrl_user/ut_automata/bin/joystick --idx 1 __name:=joystick __log:=/home/amrl_user/.ros/log/811fd94e-57b1-11eb-b3d2-521039dc703d/joystick-5.log
            ├─6885 /home/amrl_user/ut_automata/bin/gui __name:=gui __log:=/home/amrl_user/.ros/log/811fd94e-57b1-11eb-b3d2-521039dc703d/gui-6.log
            ├─6906 dbus-launch --autolaunch 1597152a6da84b2892637e4a75f64179 --binary-syntax --close-stderr
            ├─6907 /usr/bin/dbus-daemon --syslog-only --fork --print-pid 5 --print-address 7 --session
            └─7632 /home/amrl_user/ut_automata/bin/vesc_driver --config_dir=/home/amrl_user/ut_automata/config __name:=vesc_driver __log:=/home/amrl_user/.ros/log/811fd
   ```
1. Errors on boot will be logged to `syslog`. To inspect for errors:
   ```
   less /var/log/syslog
   ```

### Motion Limits

The car's motion profile is limited by the top speed, the top acceleration and deceleration, and the VESC motor counts/second. The relevant lines in the `config/vesc.lua` config file are:
   ```
   erpm_speed_limit = 22000;
   max_acceleration = 6.0; -- m/s^2
   max_deceleration = 6.0; -- m/s^2
   ```
