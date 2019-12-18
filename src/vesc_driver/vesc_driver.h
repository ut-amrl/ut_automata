// -*- mode:c++; fill-column: 100; -*-

#ifndef VESC_DRIVER_VESC_DRIVER_H_
#define VESC_DRIVER_VESC_DRIVER_H_

#include <atomic>
#include <string>

#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "boost/optional.hpp"
#include "ackermann_msgs/AckermannDriveStamped.h"
#include "f1tenth_course/AckermannCurvatureDriveMsg.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Joy.h"

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

  // limits on VESC commands
  struct CommandLimit {
    CommandLimit(
        const ros::NodeHandle& nh,
        const std::string& str,
        const boost::optional<double>& min_lower = boost::optional<double>(),
        const boost::optional<double>& max_upper = boost::optional<double>());
    double clip(double value);
    std::string name;
    boost::optional<double> lower;
    boost::optional<double> upper;
  };
  CommandLimit duty_cycle_limit_;
  CommandLimit current_limit_;
  CommandLimit brake_limit_;
  CommandLimit speed_limit_;
  CommandLimit position_limit_;
  CommandLimit servo_limit_;

  // ROS services
  ros::Publisher state_pub_;
  ros::Publisher odom_pub_;
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
    kAutonomousDrive = 2
  };


  // driver state machine mode (state)
  driver_mode_t driver_mode_;
  // Drive mode.
  DriveMode drive_mode_;
  // firmware major version reported by vesc
  int fw_version_major_;
  // firmware minor version reported by vesc
  int fw_version_minor_;

  // Conversion parameters.
  double speed_to_erpm_gain_;
  double speed_to_erpm_offset_;
  double steering_to_servo_gain_;
  double steering_to_servo_offset_;
  double wheel_base_;


  // Time of last command, for safety motion profiling
  std::atomic<double> t_last_command_;
  // Time of last joystick message, for safety.
  std::atomic<double> t_last_joystick_;
  // Last servo angle command
  double last_steering_angle_;

  // Create an odometry message
  nav_msgs::Odometry odom_msg_;

  // Convert curvature commands to steering angle.
  float CalculateSteeringAngle(float lin_vel, float rot_vel);

  // Safety profiling.
  void checkCommandTimeout();

  // Send commands to the VESC.
  void sendDriveCommands();

  // ROS callbacks
  void ackermannCmdCallback(
      const ackermann_msgs::AckermannDriveStamped::ConstPtr& cmd);
  void ackermannCurvatureCallback(
      const f1tenth_course::AckermannCurvatureDriveMsg& cmd);
  void joystickCallback(const sensor_msgs::Joy& msg);

  void updateOdometry(double rpm, double steering_angle);
};

} // namespace vesc_driver

#endif // VESC_DRIVER_VESC_DRIVER_H_
