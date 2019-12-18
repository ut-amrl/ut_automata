#include <utility>

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>

#include "shared/math/math_util.h"

#include "f1tenth_course/AckermannCurvatureDriveMsg.h"

using namespace std;
using f1tenth_course::AckermannCurvatureDriveMsg;
using math_util::Bound;

const bool kDebug = false;

// drive_enabled_ works as the emergency stop. If set to false velocity
// commands will not be sent to the motor driver by this node.
bool drive_enabled_ = true;

// In order for velocity commands to be
// sent to motor driver by this node two conditions should be met:
// 1- drive_enabled_ should be "true" (emergency stop released)
// 2- Deadman button should be engaged, i.e. have been held for more than 
//    kDeadManButtonHoldTime seconds.
const float kDeadManButtonHoldTime = 1.0; // sec
bool deadman_button_engaged_ = false;


const float kWheelBase = 0.20; // meter
const float kCmdRate = 20; // Hz
const double kDebouncingDelay = 0.4; // seconds
const float kMinimumRefreshRate = 1.0; // (Hz) if the obtained command rate is 
                                       // less than this threshold. The command 
                                       // will be reset to zero

const double kTransAccel = 0.4; // forward acceleration m/s^2 0.8
const double kTransDecel = 0.8; // forward deceleration m/s^2 def: 0.4 for JPP. 
                                // 0.8 for kinect
const double kRotAccel = 10.0;//0.05; // rotational acceleration rad/s^2

typedef geometry_msgs::Point GPoint;

pair <double, double> latest_desired_vel = make_pair<double, double>(0, 0);
double forward_vel = 0., rot_vel = 0.;

//publishers not in main
ros::Publisher vel_pub;
ros::Publisher vel_pub_ackermann;
ros::Publisher path_pub;

float CalculateSteeringAngle(float lin_vel, float rot_vel) {
  float steering_angle = 0.0;
  if (rot_vel == 0) {
    return steering_angle;
  }

  float turn_radius = lin_vel / rot_vel;
  steering_angle = atan(kWheelBase / turn_radius);
  return steering_angle;
}

void DesiredVelCallBackAckermann(const AckermannCurvatureDriveMsg& msg) {
if (!isfinite(msg.velocity) || !isfinite(msg.curvature)) {
    printf("Ignoring non-finite drive values: %f %f\n",
           msg.velocity,
           msg.curvature);
    return;
  }
  latest_desired_vel.first = msg.velocity;
  latest_desired_vel.second = msg.velocity * msg.curvature;
}

void JoystickCallback(const sensor_msgs::JoyConstPtr& msg) {
  // read joystick input
  // Wired Xbox360 Controller
  int start_btn = msg->buttons[7];
  int deadman_btn = msg->buttons[5]; // RB

  
  static double last_start_press_time = 0;
  static bool deadman_btn_pressed = false;
  static double deadman_btn_press_time = 0;
  static double deadman_btn_hold_duration = 0;
  
  if (start_btn) {
    double curr_time = ros::Time::now().toSec();
    if ((curr_time - last_start_press_time) > kDebouncingDelay) {
      drive_enabled_ = !drive_enabled_;
      last_start_press_time = ros::Time::now().toSec();
      ROS_INFO("DriveEnabled: %d\n", drive_enabled_);
    }
  }
  
  // Deadman button should have been pressed for at least kDeadManButtonHoldTime
  // in order for it to be engaged
  if (deadman_btn) {
    if (!deadman_btn_pressed) {
      deadman_btn_press_time = ros::Time::now().toSec();
      deadman_btn_pressed = true;
    } 
    
    deadman_btn_hold_duration = ros::Time::now().toSec() 
                                  - deadman_btn_press_time;
    if (deadman_btn_hold_duration > kDeadManButtonHoldTime) {
      deadman_button_engaged_ = true;
    }
  } else {
    deadman_btn_pressed = false;
    deadman_button_engaged_ = false;
    deadman_btn_hold_duration = 0.0;
  }
}

void DriveEnableCallback(const std_msgs::Bool& msg) {
  drive_enabled_ = msg.data;
}

void SendCommand() {
  const pair<double, double> desired_vel = latest_desired_vel;

  if (kDebug) {
    ROS_INFO("wanted V: %f, W: %f", desired_vel.first, desired_vel.second);
  }

  // accelerate or decelerate accordingly
  static double prev_time = 0.0;
  double curr_time = ros::Time::now().toSec();
  double del_t = curr_time - prev_time;
  // Clamp del_t to a maximum value for cases when the button has not been
  // pressed for a while
  if (del_t > (1.0 / kMinimumRefreshRate)) {
    del_t = 0.05;
    forward_vel = 0.0;
    rot_vel = 0.0;
  }
  prev_time = curr_time;

  if (drive_enabled_ && deadman_button_engaged_) {
    forward_vel = Bound(forward_vel - kTransDecel * del_t,
                        forward_vel + kTransDecel * del_t,
                        desired_vel.first);
    rot_vel = Bound(rot_vel - kRotAccel * del_t,
                    forward_vel + kRotAccel * del_t,
                    desired_vel.second);
    if (kDebug) {
      ROS_INFO("real V: %f, W: %f", forward_vel, rot_vel);
    }
    const float steering_angle = CalculateSteeringAngle(forward_vel, rot_vel);

    ackermann_msgs::AckermannDriveStamped vel_msg_ackermann;
    vel_msg_ackermann.header.stamp = ros::Time::now();
    vel_msg_ackermann.header.frame_id = "base_link";
    vel_msg_ackermann.drive.speed = forward_vel;
    vel_msg_ackermann.drive.steering_angle = steering_angle;
    vel_pub_ackermann.publish(vel_msg_ackermann);
  } else {
    forward_vel = 0.0;
    rot_vel = 0.0;
    ROS_INFO("Drive is Disabled.");
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "motion_profiler");
  ros::NodeHandle nh;

  vel_pub_ackermann = nh.advertise<ackermann_msgs::AckermannDriveStamped>(
                                "/commands/ackermann", 1);

  ros::Subscriber sub_safe_drive = 
      nh.subscribe("/bluetooth_teleop/joy", 1, JoystickCallback);
  ros::Subscriber sub_velocity_desired_ackermann =
      nh.subscribe("/ackermann_curvature_drive",
                    1,
                    DesiredVelCallBackAckermann);
  ros::Subscriber sub_drive_enable = 
      nh.subscribe("/motion_profiler/enable_drive", 1, DriveEnableCallback);

  ros::Rate rate(kCmdRate);
  while (ros::ok()) {
    ros::spinOnce();
    SendCommand();
    rate.sleep();
  }

  return 0;
}
