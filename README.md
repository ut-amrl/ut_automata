# f1tenth_course
Mono-Repository for F1/10 courses

## Build

### Update your .profile file
Add the following lines to the bottom of  ~/.profile:
`source /opt/ros/melodic/setup.bash`

`export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/f1tenth_course`

`export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/projects/cs378_starter`

After adding these lines you will need to either relog into the computer or run:
`source ~/.profile`
 
### Clone and Build Course Code
From your home directory run: 
git clone https://github.com/ut-amrl/f1tenth_course.git

`cd f1tenth_course`

`make -j`
