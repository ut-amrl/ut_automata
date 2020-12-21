# UT AUTOmata
[![Build Status](https://travis-ci.com/ut-amrl/ut_automata.svg?token=rBLDT1qXfkKmkTerGLzY&branch=master)](https://travis-ci.com/ut-amrl/ut_automata)

Infrastructure repository for UT AUTOmata

![UT AUTOmata](https://amrl.cs.utexas.edu/assets/images/robots/automata_group.jpg)

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
1. Clone the reposotiroy, including the submodules:
   ```
   git clone https://github.com/ut-amrl/ut_automata.git --recurse-submodule
   ```
   If you forgot to include the ```--recurse-submodule```
   flag when cloning the repo, you can  clone the submodules later using:
   ```
   git submodule init
   git submodule update
   ```
1. Build the repo:
   ```
   cd ut_automata
   make -j
   ```
1. To build the hardware drivers as well on the actual cars:
   ```
   make hardware
   ```

### Motion Limits

The car's motion profile is limited by the top speed, the top acceleration and deceleration, and the VESC motor counts/second. The relevant lines in the `config/vesc.lua` config file are:
   ```
   erpm_speed_limit = 22000;
   max_acceleration = 6.0; -- m/s^2
   max_deceleration = 6.0; -- m/s^2
   ```
