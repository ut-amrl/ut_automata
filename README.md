# f1tenth_course
Repository for F1/10 courses

### Overview
These instructions are tailored to the computer setup in the GDC1.310 lab. If you are setting this up on your own personal computer, you will need to modify the instructions for your own setup, including perhaps saving the setup script entries to `.bashrc` instead of `.profile`

### Update your `.profile` file
1. Add the following lines to the bottom of  `~/.profile`:
   ```
   source /opt/ros/melodic/setup.bash
   export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/f1tenth_course
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
   git clone https://github.com/ut-amrl/f1tenth_course.git
   cd f1tenth_course
   make -j
   ```
