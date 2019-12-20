
#include <ros/ros.h>
#include <ros/console.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <time.h>
#include <unistd.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <string>
#include <sstream>
#include <dirent.h>
#include <vector>

#include "gflags/gflags.h"
#include <eigen3/Eigen/Dense>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/CompressedImage.h>

DEFINE_string(out_dir, "", "Output directory for ROS bags");


#define O_RDONLY   00
#define BILLION 1000000000.0L;

using std::stringstream;
using std::string;
using std::vector;

// Constants
const double kDebouncingDelay = 0.4; // seconds
const float kWheelBase = 0.20; // meter

// Globals
string kBagFileDirectory;
string kBagNodeNameDirectory = "bag_node_name.txt";
string kLatestBagFileName;
const int kPrefixLength = 5;

bool kRecording = false;
bool kManualDriving = false;


ros::Publisher velocity_command_publisher;
ros::Publisher vel_ackermann_publisher;


float CalculateSteeringAngle(float lin_vel, float rot_vel) {
  float steering_angle = 0.0;
  if (rot_vel == 0) {
    return steering_angle;
  }
  
  float turn_radius = lin_vel / rot_vel;
  steering_angle = atan(kWheelBase / turn_radius);
  return steering_angle;
}  

void Execute(const std::string& cmd) {
  if (system(cmd.c_str()) != 0) {
    fprintf(stderr, "ERROR executing command `%s`\n", cmd.c_str());
  }
}

// Helper function to kill the bag recoring node
void StopRecording (bool discard) {
  stringstream command;
  stringstream command1;
  stringstream command2;

  // Find out the name of the node recording the bag file and save it to a file
  command1 << "rosnode list | grep /record_* > " << 
              kBagNodeNameDirectory.c_str();
  Execute(command1.str().c_str());
  std::ifstream in;
  string record_node_name;

  // Read the name of the recording node from file and kill it
  in.open (kBagNodeNameDirectory.c_str(), std::ifstream::in);
  if(!in.is_open()){
      ROS_INFO("Error openning bag_node_name.txt");
  } else {
    if (std::getline(in, record_node_name)) {
      command << "rosnode kill " << record_node_name.c_str();
      Execute(command.str().c_str());
    }
    in.close();
  }

  // Remove the temporary bag_node_name.txt file
  command2 << "rm -rf " << kBagNodeNameDirectory.c_str();
  Execute(command2.str().c_str());

  if (discard) {
    stringstream command2;

    // Take out the .bag from the end of the latest bag file
    string truncated_name = kLatestBagFileName.substr(0,
                            kLatestBagFileName.length() - 4);
    command2 << "rm " << kBagFileDirectory.c_str() << 
                truncated_name.c_str() << "*";
    Execute(command2.str().c_str());
  }
}

// Helper function that sends velocity commands to the Robot to signal the 
// start and end of the recording 
void MoveRobot (bool direction, double duration) {
  struct timespec ms1;
  struct timespec ms2;
  double s_elapsed1;

  geometry_msgs::Twist vel_command;
  vel_command.linear.x = 0;
  vel_command.linear.y = 0;
  vel_command.linear.z = 0;
  vel_command.angular.x = 0;
  vel_command.angular.y = 0;
  // float rot_vel = 0.25;
  float lin_vel = 0.25;

  // rotate left
  if (direction) {
    // vel_command.angular.z = rot_vel;
    vel_command.linear.x = -lin_vel;
    
  // rotate right
  } else {
    // vel_command.angular.z = -rot_vel;
    vel_command.linear.x = lin_vel;
  }

  // Generate the ackermann drive message for the corresponding
  // linear and rotational velocity values
  ackermann_msgs::AckermannDriveStamped vel_msg_ackermann;
  vel_msg_ackermann.header.stamp = ros::Time::now();
  vel_msg_ackermann.header.frame_id = "base_link";
  
  float steering_angle = CalculateSteeringAngle(vel_command.linear.x, 0.0);
  
  vel_msg_ackermann.drive.speed = vel_command.linear.x;
  vel_msg_ackermann.drive.steering_angle = steering_angle;

  clock_gettime(CLOCK_MONOTONIC, &ms1);
  clock_gettime(CLOCK_MONOTONIC, &ms2);
  s_elapsed1 = (ms2.tv_sec - ms1.tv_sec) + (ms2.tv_nsec - ms1.tv_nsec) /
                BILLION;

  while (s_elapsed1 < duration) {
    velocity_command_publisher.publish(vel_command);
    vel_ackermann_publisher.publish(vel_msg_ackermann);
    clock_gettime(CLOCK_MONOTONIC, &ms2);
    s_elapsed1 = (ms2.tv_sec - ms1.tv_sec) + (ms2.tv_nsec - ms1.tv_nsec) /
                  BILLION;
  }

}

// Helper function that reads the names of the files in the bag files directory
// and figures out the name of the next bag file to be saved
void FindNextFileName (std::string* file_name) {
  char numbering[5];
  DIR* dirp = opendir(kBagFileDirectory.c_str());
  struct dirent * dp;
  int max_prefix_number = 0;

  while ((dp = readdir(dirp)) != NULL){

    // Ignore the '.' and ".." directories
    if(!strcmp(dp->d_name, ".") || !strcmp(dp->d_name, "..")) continue;
    for(int i = 0; i < kPrefixLength ; i++){
        numbering[i] = dp->d_name[i];
    }

    int prefix_number = atoi(numbering);
    if(prefix_number > max_prefix_number) max_prefix_number = prefix_number;
  }
  (void)closedir(dirp);

  char cur_prefix_number [6];
  sprintf(cur_prefix_number, "%05d", max_prefix_number + 1);
  *file_name = cur_prefix_number;

  file_name->append(".bag");
}

// Helper function which starts and stops recording bag files by reading the
// corresponding signals from the joystick
void JoystickCallback(const sensor_msgs::Joy::ConstPtr& joy) {
  static bool one_time_rec = false;
  int select_btn = joy->buttons[6]; // back button on XBOX360
  int B = joy->buttons[1];
  int l2 = joy->axes[2] > 0.9;
  int r2 = joy->axes[5] > 0.9;
  int r1 = joy->buttons[5];

  // Toggling the human intervention session.
  // It publishes to a topic to announce the start and end of each human
  // intervention session
  static double last_select_press_time = 0;
  
  if (select_btn) {
    double curr_time = ros::Time::now().toSec();
    if ((curr_time - last_select_press_time) > kDebouncingDelay) {
      last_select_press_time = ros::Time::now().toSec();
    }
  }

  // If X pressed, start recording
  if (r2 && l2 && r1) {
    if (!kRecording) {
      kRecording = true;
      kManualDriving = true;
      one_time_rec = true;
    }
  }

  // If B is pressed, stop recording
  if (B) {
    if (kRecording) {
      kRecording = false;
      one_time_rec = false;
      
      if (kManualDriving) {
        // Save the recording
        StopRecording(false);
        MoveRobot(true, 1.0);
      } else {
        // Discard the recording
        StopRecording(true);
      }

      kManualDriving = false;
      ROS_INFO("Recording status: %d", kRecording);
    }
  }

  if (one_time_rec) {
    ROS_INFO("Recording status: %d", kRecording);
    one_time_rec = false;
    string bag_file_name;
    stringstream kRecording_command;
    FindNextFileName(&bag_file_name);
    kLatestBagFileName = bag_file_name;

    kRecording_command << "rosbag record --split --size=10000 -o "
                       << kBagFileDirectory.c_str() 
                       << bag_file_name.c_str() <<    
    // *** Intel Realsense
    " /realsense/color/image_raw/compressed "
    // " /real_ff/depth/image_rect_raw "
    // " /real_ff/laserscan "

    // *** Human intervention signal for ENML
    " /initialpose "
    " /human_intervention/status "
    
    // *** Localization information
    " /enml/localization "
    " /enml/lost_metric "
    
    // *** Goal Information
    " /localization_gui/nav_goal "
    " /kinect_local_nav/goal "

    // *** Ydlidar
    " /scan "

    " /odom  &";

    if (kManualDriving) {
      MoveRobot(false, 1.0);
    }
  }
}


int main(int argc, char **argv) {
  ros::init(argc, argv, "logger");
  ros::NodeHandle nh;
  gflags::ParseCommandLineFlags(&argc, &argv, true);


  if (FLAGS_out_dir == "") {
    ROS_ERROR("Bag file directory was not set");
    return -1;
  }


  ROS_INFO("Bag file directory: %s", FLAGS_out_dir.c_str());

  ros::Subscriber joystick_subscriber = 
    nh.subscribe("/joystick",5, JoystickCallback);


  velocity_command_publisher =  nh.advertise<geometry_msgs::Twist>("/cmd_vel", 
                                5);
  vel_ackermann_publisher = nh.advertise<ackermann_msgs::AckermannDriveStamped>
                      ("/commands/ackermann", 1);
  
  ROS_INFO("Press the R1 + R2 + L2 to start logging data."
          " Press O to stop recording.\n");
  ROS_INFO("Press the select button to toggle the human intervention mode\n");
  ROS_INFO("Hold down R2 + L2 to check availability of camera stream and "
           "odometry. The robot will move if no messages are being "
           "published to these topics.\n");
  
  ros::Rate loop_rate(10);
  while(ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
