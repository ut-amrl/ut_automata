# ut_automata
Repository for UT AUTOmata courses  

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

### Motion Limits

The car's motion profile is limited by the top speed, the top acceleration and deceleration, and the VESC motor counts/second. The relevant lines in the `config/vesc.lua` config file are:
   ```
   erpm_speed_limit = 22000;
   max_acceleration = 6.0; -- m/s^2
   max_deceleration = 6.0; -- m/s^2
   ```
