// -*- mode:c++; fill-column: 100; -*-

#ifndef VESC_DRIVER_VESC_DRIVER_H_
#define VESC_DRIVER_VESC_DRIVER_H_

#include <atomic>
#include <string>

#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "boost/optional.hpp"
#include "amrl_msgs/AckermannCurvatureDriveMsg.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Joy.h"
#include "std_msgs/Bool.h"

#include "vesc_driver/vesc_interface.h"
#include "vesc_driver/vesc_packet.h"

namespace vesc_driver
{

class VescDriver
{
public:

  VescDriver(ros::NodeHandle nh,
             ros::NodeHandle private_nh);

private:
  // interface to the VESC
  VescInterface vesc_;
  void vescPacketCallback(const boost::shared_ptr<VescPacket const>& packet);
  void vescErrorCallback(const std::string& error);
  void timerCallback(const ros::SteadyTimerEvent& event);

  // ROS services
  ros::Publisher state_pub_;
  ros::Publisher odom_pub_;
  ros::Publisher car_status_pub_;
  ros::Publisher autonomy_enabler_pub_;
  ros::Subscriber ackermann_curvature_sub_;
  ros::Subscriber joystick_sub_;
  ros::SteadyTimer timer_;

  // driver modes (possible states)
  typedef enum {
    MODE_INITIALIZING,
    MODE_OPERATING
  } driver_mode_t;

  // Drive mode
  enum DriveMode {
    kStoppedDrive = 0,
    kJoystickDrive = 1,
    kAutonomousDrive = 2,
    kAutonomousContinuousDrive = 3
  };

  // treat as keypress event
  enum JoyPress {
    kNoToggle = 0,
    kToggleOn = 1,
    kToggleOff = 2
  };

  // driver state machine mode (state)
  driver_mode_t driver_mode_;
  // Drive mode.
  DriveMode drive_mode_;
  // firmware major version reported by vesc
  int fw_version_major_;
  // firmware minor version reported by vesc
  int fw_version_minor_;

  // status of previous toggle button;
  int prevToggleState_;

  // Time of last command, for safety motion profiling
  std::atomic<double> t_last_command_;
  // Time of last joystick message, for safety.
  std::atomic<double> t_last_joystick_;
  // Last servo angle command
  float last_steering_angle_;

  // Create an odometry message
  nav_msgs::Odometry odom_msg_;

  // Convert curvature commands to steering angle.
  float CalculateSteeringAngle(float lin_vel, float rot_vel);

  // Safety profiling.
  void checkCommandTimeout();

  // Send commands to the VESC.
  void sendDriveCommands();

  // ROS callbacks
  void ackermannCurvatureCallback(
      const amrl_msgs::AckermannCurvatureDriveMsg& cmd);
  void joystickCallback(const sensor_msgs::Joy& msg);

  void updateOdometry(float rpm, float steering_angle);
  
  // true if car is running autonomously, false otherwise 
  bool isAutonomous();
  // report current state of the autonomy toggle button
  int toggleState(int curToggleState);
};

} // namespace vesc_driver

#endif // VESC_DRIVER_VESC_DRIVER_H_
