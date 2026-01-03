// -*- mode:c++; fill-column: 100; -*-

#ifndef VESC_DRIVER_VESC_DRIVER_H_
#define VESC_DRIVER_VESC_DRIVER_H_

#include <atomic>
#include <string>
#include <memory>
#include <fstream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "boost/optional.hpp"
#include "amrl_msgs/msg/ackermann_curvature_drive_msg.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "ut_automata/msg/vesc_state_stamped.hpp"
#include "ut_automata/msg/car_status_msg.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include "vesc_driver/vesc_interface.h"
#include "vesc_driver/vesc_packet.h"
#include "vesc_driver/ekf_fusion.h"
#include "mpu6050driver/mpu6050sensor.h"
#include "ut_automata/msg/vesc_state_stamped.hpp"
#include "ut_automata/msg/car_status_msg.hpp"

namespace vesc_driver
{

class VescDriver
{
public:

  VescDriver(rclcpp::Node::SharedPtr nh,
             rclcpp::Node::SharedPtr private_nh);

private:
  // interface to the VESC
  VescInterface vesc_;
  void vescPacketCallback(const boost::shared_ptr<VescPacket const>& packet);
  void timerCallback();

  // node handles
  rclcpp::Node::SharedPtr nh_;
  rclcpp::Node::SharedPtr private_nh_;

  // ROS services
  rclcpp::Publisher<ut_automata::msg::VescStateStamped>::SharedPtr state_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr drive_pub_;
  rclcpp::Publisher<ut_automata::msg::CarStatusMsg>::SharedPtr car_status_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr autonomy_enabler_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Subscription<amrl_msgs::msg::AckermannCurvatureDriveMsg>::SharedPtr ackermann_curvature_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joystick_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr override_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

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
  // Last smoothed speed actually sent to motor (for EKF)
  float last_smooth_speed_;
  
  // Last D-pad state for trim control
  int last_dpad_x_;
  int last_dpad_y_;
  
  // Runtime-adjustable trim offsets (initialized from config)
  float steering_offset_trim_;
  float speed_offset_trim_;
  
  // Override status flags
  bool override_drive_active_ = false;
  bool override_steer_active_ = false;

  // Create an odometry message
  nav_msgs::msg::Odometry odom_msg_;

  // EKF for IMU fusion
  std::unique_ptr<EKFFusion> ekf_;
  std::unique_ptr<MPU6050Sensor> mpu6050_;
  bool fuse_imu_ = true;
  bool imu_available_;
  std::vector<float> imu_to_car_transform_;

  // Debug logging
  std::ofstream debug_log_;

  // Convert curvature commands to steering angle.
  float CalculateSteeringAngle(float lin_vel, float rot_vel);

  // Safety profiling.
  void checkCommandTimeout();

  // Send commands to the VESC.
  void sendDriveCommands();

  // ROS callbacks
  void ackermannCurvatureCallback(
      const amrl_msgs::msg::AckermannCurvatureDriveMsg::SharedPtr cmd);
  void joystickCallback(const sensor_msgs::msg::Joy::SharedPtr msg);

  void updateOdometry(float rpm, float tachometer, float steering_angle);
  
  // Save trim offsets to car.lua configuration file
  void saveTrimOffsetsToConfig();
  
  // true if car is running autonomously, false otherwise 
  bool isAutonomous();
  // report current state of the autonomy toggle button
  int toggleState(int curToggleState);
};

} // namespace vesc_driver

#endif // VESC_DRIVER_VESC_DRIVER_H_
