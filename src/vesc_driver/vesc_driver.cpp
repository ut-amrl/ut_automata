// -*- mode:c++; fill-column: 100; -*-

#include "vesc_driver.h"

#include <atomic>
#include <cassert>
#include <cmath>
#include <sstream>
#include <fstream>
#include <iomanip>
#include <unistd.h>
#include <regex>
#include <ctime>

#include "boost/bind/bind.hpp"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "ut_automata/msg/car_status_msg.hpp"
#include "ut_automata/msg/vesc_state_stamped.hpp"
#include "amrl_msgs/msg/ackermann_curvature_drive_msg.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "config_reader/config_reader.h"
#include "shared/math/math_util.h"

using namespace boost::placeholders;

static const bool kDebug = false;
static const float kCommandRate = 20;
static const float kCommandInterval = 1.0 / kCommandRate;

CONFIG_FLOAT(speed_to_erpm_gain_, "speed_to_erpm_gain");
CONFIG_FLOAT(speed_to_erpm_offset_, "speed_to_erpm_offset");
CONFIG_FLOAT(steering_to_servo_gain_, "steering_angle_to_servo_gain");
CONFIG_FLOAT(steering_to_servo_offset_, "steering_angle_to_servo_offset");
CONFIG_FLOAT(wheelbase_, "wheelbase");
CONFIG_FLOAT(erpm_speed_limit_, "erpm_speed_limit");
CONFIG_FLOAT(servo_min_, "servo_min");
CONFIG_FLOAT(servo_max_, "servo_max");
CONFIG_FLOAT(max_accel_, "max_acceleration");
CONFIG_FLOAT(max_decel_, "max_deceleration");
CONFIG_FLOAT(turbo_speed_, "joystick_turbo_speed");
CONFIG_FLOAT(normal_speed_, "joystick_normal_speed");
CONFIG_FLOAT(max_steering_angle_, "max_steering_angle");
CONFIG_FLOAT(steering_curve_xm_, "steering_curve_xm");
CONFIG_FLOAT(steering_curve_ym_, "steering_curve_ym");
CONFIG_STRING(joystick_mode_, "joystick_mode");
CONFIG_STRING(serial_port_, "serial_port");
CONFIG_BOOL(fuse_imu_, "fuse_imu");
CONFIG_INT(i2c_bus_number_, "i2c_bus_number");
CONFIG_BOOL(calibrate_imu_, "calibrate_imu");
CONFIG_INT(imu_gyro_range_, "imu_gyro_range");
CONFIG_INT(imu_accel_range_, "imu_accel_range");
CONFIG_INT(imu_dlpf_bandwidth_, "imu_dlpf_bandwidth");
CONFIG_BOOL(instant_override_, "instant_override");
CONFIG_BOOL(debug_ekf_, "debug_ekf");
CONFIG_STRING(debug_log_path_, "debug_log_path");

DEFINE_string(config_dir, "/home/orin/roboracer_ws/src/ut_automata/config", 
    "Directory containing the car.lua and vesc.lua config files.");

using ut_automata::msg::CarStatusMsg;
using ut_automata::msg::VescStateStamped;

namespace {

float mux_drive_speed_ = 0;
float mux_steering_angle_ = 0;


VescStateStamped state_msg_;
CarStatusMsg car_status_msg_;

// Helper for 4th degree Bezier
float Bezier4(float t, float p0, float p1, float p2, float p3, float p4) {
  float one_minus_t = 1.0f - t;
  float one_minus_t2 = one_minus_t * one_minus_t;
  float one_minus_t3 = one_minus_t2 * one_minus_t;
  float one_minus_t4 = one_minus_t3 * one_minus_t;
  float t2 = t * t;
  float t3 = t2 * t;
  float t4 = t3 * t;
  
  return one_minus_t4 * p0 +
         4.0f * one_minus_t3 * t * p1 +
         6.0f * one_minus_t2 * t2 * p2 +
         4.0f * one_minus_t * t3 * p3 +
         t4 * p4;
}

// Derivative of 4th degree Bezier
float Bezier4Prime(float t, float p0, float p1, float p2, float p3, float p4) {
  float one_minus_t = 1.0f - t;
  float one_minus_t2 = one_minus_t * one_minus_t;
  float one_minus_t3 = one_minus_t2 * one_minus_t;
  float t2 = t * t;
  float t3 = t2 * t;
  
  return 4.0f * one_minus_t3 * (p1 - p0) +
         12.0f * one_minus_t2 * t * (p2 - p1) +
         12.0f * one_minus_t * t2 * (p3 - p2) +
         4.0f * t3 * (p4 - p3);
}

}  // namespace

namespace vesc_driver
{

VescDriver::VescDriver(rclcpp::Node::SharedPtr nh,
           rclcpp::Node::SharedPtr private_nh) :
    vesc_(std::string(),
      boost::bind(&VescDriver::vescPacketCallback, this, _1)),
    nh_(nh), private_nh_(private_nh),
    driver_mode_(MODE_INITIALIZING),
    drive_mode_(kStoppedDrive),
    fw_version_major_(-1),
    fw_version_minor_(-1),
    t_last_command_(0),
    t_last_joystick_(0),
    last_smooth_speed_(0),
    last_dpad_x_(0),
    last_dpad_y_(0),
    steering_offset_trim_(0),
    speed_offset_trim_(0),
    imu_available_(false) {
  // Load config. Ensure car.lua exists; if it doesn't, create it using
  // the hostname. Hostnames like "orin07" will produce car_name = "car07".
  std::string car_path = FLAGS_config_dir + "/car.lua";
  {
    std::ifstream car_in(car_path);
    if (!car_in.good()) {
      char hn[256] = {0};
      if (gethostname(hn, sizeof(hn)) != 0) {
        LOG(WARNING) << "Failed to get hostname; defaulting car number to 00";
      }
      std::string hs(hn);
      std::smatch m;
      std::regex r("([0-9]+)$");
      std::string digits = "00";
      if (std::regex_search(hs, m, r) && m.size() > 1) {
        digits = m[1];
      }
      std::string car_name = "car" + digits;
      std::ofstream car_out(car_path);
      if (car_out) {
        car_out << "car_name = \"" << car_name << "\";\n";
        car_out.close();
        LOG(INFO) << "Generated " << car_path << " with car_name " << car_name;
      } else {
        LOG(WARNING) << "Unable to create " << car_path << "; proceeding without it.";
      }
    }
  }

  RCLCPP_INFO(nh_->get_logger(), "Loading configuration files:");
  RCLCPP_INFO(nh_->get_logger(), "  car.lua: %s", car_path.c_str());
  RCLCPP_INFO(nh_->get_logger(), "  vesc.lua: %s", (FLAGS_config_dir + "/vesc.lua").c_str());
  RCLCPP_INFO(nh_->get_logger(), "  joystick.lua: %s", (FLAGS_config_dir + "/joystick.lua").c_str());
  
  // DEBUG: Check value before config reader
  RCLCPP_INFO(nh_->get_logger(), "BEFORE ConfigReader: fuse_imu = %s", fuse_imu_ ? "true" : "false");
  
  config_reader::ConfigReader reader({
    FLAGS_config_dir + "/vesc.lua",
    car_path,
    FLAGS_config_dir + "/joystick.lua",
  });
  
  // IMU to car frame transformation
  // Format: [x_sign, x_axis, y_sign, y_axis, z_sign, z_axis]
  // Car frame: +x forward, +y left, +z up
  // Axis indices: 0=x, 1=y, 2=z
  // 
  // Common IMU mounting orientations:
  // 1. Standard (IMU +x forward, +y left): {1, 0, 1, 1, 1, 2}
  // 2. IMU +x rear, +y right (user's setup): {-1, 0, -1, 1, -1, 0}
  // 3. IMU +x right, +y forward: {1, 1, -1, 0, 1, 2}
  // 
  // TODO: Read from config file when LuaScript supports arrays
  // For now, set based on your IMU mounting orientation:
  imu_to_car_transform_ = {-1, 0, -1, 1, -1, 0};  // IMU: +x rear, +y right
  
  RCLCPP_INFO(nh_->get_logger(), "IMU to car transform: [%.0f,%d, %.0f,%d, %.0f,%d]",
              imu_to_car_transform_[0], (int)imu_to_car_transform_[1],
              imu_to_car_transform_[2], (int)imu_to_car_transform_[3],
              imu_to_car_transform_[4], (int)imu_to_car_transform_[5]);
  
  // DEBUG: Check value immediately after config reader
  RCLCPP_INFO(nh_->get_logger(), "AFTER ConfigReader: fuse_imu = %s", fuse_imu_ ? "true" : "false");
  
  // Debug: Print ALL configuration values loaded from Lua files
  RCLCPP_INFO(nh_->get_logger(), "=== Configuration Values Loaded ===");
  RCLCPP_INFO(nh_->get_logger(), "From vesc.lua:");
  RCLCPP_INFO(nh_->get_logger(), "  serial_port = %s", serial_port_.c_str());
  RCLCPP_INFO(nh_->get_logger(), "  speed_to_erpm_gain = %.2f", speed_to_erpm_gain_);
  RCLCPP_INFO(nh_->get_logger(), "  speed_to_erpm_offset = %.2f", speed_to_erpm_offset_);
  RCLCPP_INFO(nh_->get_logger(), "  steering_angle_to_servo_gain = %.4f", steering_to_servo_gain_);
  RCLCPP_INFO(nh_->get_logger(), "  steering_angle_to_servo_offset = %.2f", steering_to_servo_offset_);
  RCLCPP_INFO(nh_->get_logger(), "  erpm_speed_limit = %.2f", erpm_speed_limit_);
  RCLCPP_INFO(nh_->get_logger(), "  servo_min = %.2f", servo_min_);
  RCLCPP_INFO(nh_->get_logger(), "  servo_max = %.2f", servo_max_);
  RCLCPP_INFO(nh_->get_logger(), "  wheelbase = %.3f", wheelbase_);
  RCLCPP_INFO(nh_->get_logger(), "  max_acceleration = %.2f", max_accel_);
  RCLCPP_INFO(nh_->get_logger(), "  max_deceleration = %.2f", max_decel_);
  RCLCPP_INFO(nh_->get_logger(), "  fuse_imu = %s", fuse_imu_ ? "true" : "false");
  RCLCPP_INFO(nh_->get_logger(), "  i2c_bus_number = %d", i2c_bus_number_);
  RCLCPP_INFO(nh_->get_logger(), "  calibrate_imu = %s", calibrate_imu_ ? "true" : "false");
  RCLCPP_INFO(nh_->get_logger(), "  imu_gyro_range = %d", imu_gyro_range_);
  RCLCPP_INFO(nh_->get_logger(), "  imu_accel_range = %d", imu_accel_range_);
  RCLCPP_INFO(nh_->get_logger(), "  imu_dlpf_bandwidth = %d", imu_dlpf_bandwidth_);
  RCLCPP_INFO(nh_->get_logger(), "  debug_ekf = %s", debug_ekf_ ? "true" : "false");
  if (debug_ekf_) {
    RCLCPP_INFO(nh_->get_logger(), "  debug_log_path = %s", debug_log_path_.c_str());
  }
  RCLCPP_INFO(nh_->get_logger(), "From joystick.lua:");
  RCLCPP_INFO(nh_->get_logger(), "  joystick_normal_speed = %.2f", normal_speed_);
  RCLCPP_INFO(nh_->get_logger(), "  joystick_turbo_speed = %.2f", turbo_speed_);
  RCLCPP_INFO(nh_->get_logger(), "  joystick_mode = %s", joystick_mode_.c_str());
  RCLCPP_INFO(nh_->get_logger(), "  max_steering_angle = %.3f rad (%.1f deg)", 
              max_steering_angle_, max_steering_angle_ * 180.0 / M_PI);
  RCLCPP_INFO(nh_->get_logger(), "  steering_curve_xm = %.2f", steering_curve_xm_);
  RCLCPP_INFO(nh_->get_logger(), "  steering_curve_ym = %.2f", steering_curve_ym_);
  RCLCPP_INFO(nh_->get_logger(), "  calculated min_turning_radius = %.3f m", 
              wheelbase_ / tan(max_steering_angle_));
  RCLCPP_INFO(nh_->get_logger(), "===================================");
  
  // Initialize trim offsets from config values
  steering_offset_trim_ = steering_to_servo_offset_;
  speed_offset_trim_ = speed_to_erpm_offset_;
  
  
  state_msg_.header.frame_id = "base_link";
  
  state_msg_.header.frame_id = "base_link";
  car_status_msg_.header = state_msg_.header;

  // set header stamp from node clock
  {
    uint64_t nsec = nh_->get_clock()->now().nanoseconds();
    odom_msg_.header.stamp.sec = static_cast<int32_t>(nsec / 1000000000ULL);
    odom_msg_.header.stamp.nanosec = static_cast<uint32_t>(nsec % 1000000000ULL);
  }
  odom_msg_.header.frame_id = "odom";
  odom_msg_.child_frame_id = "base_link";

  odom_msg_.twist.twist.linear.x = 0;
  odom_msg_.twist.twist.linear.y = 0;
  odom_msg_.twist.twist.linear.z = 0;
  odom_msg_.twist.twist.angular.x = 0;
  odom_msg_.twist.twist.angular.y = 0;
  odom_msg_.twist.twist.angular.z = 0;
  odom_msg_.twist.covariance =
                {0.001, 0.0, 0.0, 0.0, 0.0, 0.0,
                  0.0, 0.001, 0.0, 0.0, 0.0, 0.0,
                  0.0, 0.0, 0.001, 0.0, 0.0, 0.0,
                  0.0, 0.0, 0.0, 1000000.0, 0.0, 0.0,
                  0.0, 0.0, 0.0, 0.0, 1000000.0, 0.0,
                  0.0, 0.0, 0.0, 0.0, 0.0, 0.03};

  odom_msg_.pose.pose.position.x = 0;
  odom_msg_.pose.pose.position.y = 0;
  odom_msg_.pose.pose.position.z = 0;
  odom_msg_.pose.covariance =
                {0.001, 0.0, 0.0, 0.0, 0.0, 0.0,
                  0.0, 0.001, 0.0, 0.0, 0.0, 0.0,
                  0.0, 0.0, 1000000.0 , 0.0, 0.0, 0.0,
                  0.0, 0.0, 0.0, 1000000.0, 0.0, 0.0,
                  0.0, 0.0, 0.0, 0.0, 1000000.0, 0.0,
                  0.0, 0.0, 0.0, 0.0, 0.0, 0.03};
  odom_msg_.pose.pose.orientation.w = 1;
  odom_msg_.pose.pose.orientation.x = 0;
  odom_msg_.pose.pose.orientation.y = 0;
  odom_msg_.pose.pose.orientation.z = 0;


  // attempt to connect to the serial port
  if (kDebug) printf("CONNECT\n");
  CHECK(vesc_.connect(serial_port_)) << "Failed to connect to the VESC";
  if (kDebug) printf("CONNECTED\n");

  state_pub_ = nh_->create_publisher<ut_automata::msg::VescStateStamped>("sensors/core", rclcpp::QoS(10));
  autonomy_enabler_pub_ = nh_->create_publisher<std_msgs::msg::Bool>("autonomy_enabler", rclcpp::QoS(10));
  override_pub_ = nh_->create_publisher<std_msgs::msg::Bool>("/override_active", rclcpp::QoS(10));
  odom_pub_ = nh_->create_publisher<nav_msgs::msg::Odometry>("odom", rclcpp::QoS(10));
  drive_pub_ = nh_->create_publisher<geometry_msgs::msg::TwistStamped>("vesc_drive", rclcpp::QoS(10));
  car_status_pub_ = nh_->create_publisher<ut_automata::msg::CarStatusMsg>("car_status", rclcpp::QoS(10));
  imu_pub_ = nh_->create_publisher<sensor_msgs::msg::Imu>("imu", rclcpp::QoS(10));
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(nh_);

  ackermann_curvature_sub_ = nh_->create_subscription<amrl_msgs::msg::AckermannCurvatureDriveMsg>(
    "/ackermann_curvature_drive", rclcpp::QoS(10),
    std::bind(&VescDriver::ackermannCurvatureCallback, this, std::placeholders::_1));
  joystick_sub_ = nh_->create_subscription<sensor_msgs::msg::Joy>(
    "joystick", rclcpp::QoS(10), std::bind(&VescDriver::joystickCallback, this, std::placeholders::_1));

  // Initialize EKF and MPU6050 if IMU fusion is enabled
  if (fuse_imu_) {
    ekf_ = std::make_unique<EKFFusion>(wheelbase_);
    
    try {
      // Initialize MPU6050 sensor
      mpu6050_ = std::make_unique<MPU6050Sensor>(i2c_bus_number_);
      
      // Configure sensor
      mpu6050_->setGyroscopeRange(static_cast<MPU6050Sensor::GyroRange>(imu_gyro_range_));
      mpu6050_->setAccelerometerRange(static_cast<MPU6050Sensor::AccelRange>(imu_accel_range_));
      mpu6050_->setDlpfBandwidth(static_cast<MPU6050Sensor::DlpfBandwidth>(imu_dlpf_bandwidth_));
      
      // Calibrate if requested
      if (calibrate_imu_) {
        RCLCPP_INFO(nh_->get_logger(), "Calibrating MPU6050...");
        mpu6050_->calibrate();
      }
      
      mpu6050_->printConfig();
      mpu6050_->printOffsets();
      imu_available_ = true;
      RCLCPP_INFO(nh_->get_logger(), "IMU fusion enabled with direct MPU6050 sensor reading on I2C bus %d", i2c_bus_number_);
    } catch (const std::exception& e) {
      RCLCPP_ERROR(nh_->get_logger(), "Failed to initialize MPU6050: %s. IMU fusion disabled.", e.what());
      imu_available_ = false;
      mpu6050_.reset();
    }
  } else {
    RCLCPP_INFO(nh_->get_logger(), "IMU fusion disabled");
  }

  // Initialize debug logging if enabled
  if (debug_ekf_) {
    // Create debug directory if it doesn't exist
    std::string debug_dir = debug_log_path_.substr(0, debug_log_path_.find_last_of('/'));
    std::string mkdir_cmd = "mkdir -p " + debug_dir;
    if (system(mkdir_cmd.c_str()) != 0) {
      RCLCPP_WARN(nh_->get_logger(), "Failed to create debug directory: %s", debug_dir.c_str());
    }
    
    debug_log_.open(debug_log_path_, std::ios::out | std::ios::trunc);
    if (debug_log_.is_open()) {
      // Write CSV header
      debug_log_ << "timestamp,dt,commanded_velocity,steering_angle,"
                 << "rpm,tachometer,delta_tach,tach_to_meters,"
                 << "imu_angular_velocity_raw,imu_angular_velocity_transformed,"
                 << "ekf_x,ekf_y,ekf_theta,ekf_v,ekf_omega,"
                 << "odom_x,odom_y,odom_theta,odom_v,odom_omega\n";
      RCLCPP_INFO(nh_->get_logger(), "EKF debug logging enabled: %s", debug_log_path_.c_str());
    } else {
      RCLCPP_ERROR(nh_->get_logger(), "Failed to open debug log file: %s", debug_log_path_.c_str());
    }
  }

  if (kDebug) printf("TIMER START\n");
  // create a 50Hz timer, used for state machine & polling VESC telemetry
  timer_ = nh_->create_wall_timer(
      std::chrono::duration<double>(kCommandInterval),
      std::bind(&VescDriver::timerCallback, this));
  if (kDebug) printf("DONE INIT\n");
}

void VescDriver::checkCommandTimeout() {
  static const double kTimeout = 0.5;
  const double t_now = rclcpp::Clock(RCL_ROS_TIME).now().seconds();
  if ((t_now > t_last_command_ + kTimeout && isAutonomous()) ||
      t_now > t_last_joystick_ + kTimeout) {
    mux_drive_speed_ = 0;
    mux_steering_angle_ = 0;
  }
}

geometry_msgs::msg::TwistStamped createTwist(float velocity, float curvature) {
  geometry_msgs::msg::TwistStamped  twist_msg;
  // stamp with current ROS time
  uint64_t nsec = rclcpp::Clock(RCL_ROS_TIME).now().nanoseconds();
  twist_msg.header.stamp.sec = static_cast<int32_t>(nsec / 1000000000ULL);
  twist_msg.header.stamp.nanosec = static_cast<uint32_t>(nsec % 1000000000ULL);
  twist_msg.twist.linear.x = velocity;
  twist_msg.twist.linear.y = 0;
  twist_msg.twist.linear.z = 0;
  twist_msg.twist.angular.x = 0;
  twist_msg.twist.angular.y = 0;
  twist_msg.twist.angular.z = velocity * curvature;
  return twist_msg;
}

geometry_msgs::msg::TwistStamped CalculateDriveCmd(float speed, float steering_angle) {
  float velocity = speed;
  float turn_radius = steering_angle != 0 ? wheelbase_ / tan(steering_angle) : 0;
  float curvature = turn_radius != 0 ?  1.0 / turn_radius : 0;

  return createTwist(velocity, curvature);
}

void VescDriver::joystickCallback(const sensor_msgs::msg::Joy::SharedPtr msg) {
  static const bool kDebug = false;
  static const float kAxesEps = 0.2;
  static const size_t kManualDriveButton = 4;
  static const size_t kAutonomousDriveButton = 5;
  static const size_t kAutonomousDriveToggleButton = 7;
  if (msg->buttons.size() < 6) return;

  // Trim logic using D-pad (axes 6 and 7)
  if (msg->axes.size() >= 8) {
    int dpad_x = static_cast<int>(msg->axes[6]);
    int dpad_y = static_cast<int>(msg->axes[7]);
    
    // Steering trim (Left/Right)
    if (dpad_x != 0 && dpad_x != last_dpad_x_) {
      // Left (-1) decreases offset, Right (+1) increases offset
      float step = 0.01f;
      steering_offset_trim_ += dpad_x * step;
      saveTrimOffsetsToConfig();
      RCLCPP_INFO(nh_->get_logger(), "Steering Trim: %.3f (delta from config: %.3f)", 
                  steering_offset_trim_, steering_offset_trim_ - steering_to_servo_offset_);
    }

    // Speed trim (Up/Down)
    if (dpad_y != 0 && dpad_y != last_dpad_y_) {
      // Up (+1) increases offset, Down (-1) decreases offset
      // ERPM values are typically in thousands, so 100 is a small adjustment
      float step = 100.0f;
      speed_offset_trim_ += dpad_y * step;
      saveTrimOffsetsToConfig();
      RCLCPP_INFO(nh_->get_logger(), "Speed Trim: %.2f (delta from config: %.2f)", 
                  speed_offset_trim_, speed_offset_trim_ - speed_to_erpm_offset_);
    }
    
    last_dpad_x_ = dpad_x;
    last_dpad_y_ = dpad_y;
  }



  t_last_joystick_ = rclcpp::Clock(RCL_ROS_TIME).now().seconds();
  int toggle = toggleState(msg->buttons[kAutonomousDriveToggleButton]);

  // determine if any button/axes is pressed.
  bool pressed = (drive_mode_ == kAutonomousContinuousDrive 
    && toggle == kToggleOn);
  for(size_t i = 0; i < msg->buttons.size(); i++){
    if(i != kAutonomousDriveToggleButton && msg->buttons[i]){
      pressed = true;
    }
  }
  for(size_t i = 0; i < msg->axes.size(); i++){
    float axes_value = msg->axes[i];
    if(i == 2 || i == 5){
      // axes 2, 5's initial value is around -1.0
      axes_value += 1.0;
    }
    if(std::abs(axes_value) > kAxesEps){
      pressed = true;    
    }
  }
  if (drive_mode_ == kAutonomousContinuousDrive && pressed){
    // stop the car if any button is pressed
    drive_mode_ = kStoppedDrive;
    mux_drive_speed_ = 0;
    mux_steering_angle_ = 0;
  } else if (msg->buttons[kManualDriveButton] == 1) {
    // joystick mode
    if(kDebug) printf("Joystick\n");
    drive_mode_ = kJoystickDrive;
  } else if ((toggle == kToggleOn) ||
    (drive_mode_ == kAutonomousContinuousDrive && 
    toggle != kToggleOn)){
    if(kDebug) printf("ContAutonomous\n");
    drive_mode_ = kAutonomousContinuousDrive;
  } else if (msg->buttons[kAutonomousDriveButton] == 1) {
    if(kDebug) printf("Autonomous\n");
    drive_mode_ = kAutonomousDrive;
  } else {
    if(kDebug) printf("Stopped\n");
    drive_mode_ = kStoppedDrive;
    mux_drive_speed_ = 0;
    mux_steering_angle_ = 0;
  }
  if (drive_mode_ == kJoystickDrive) {
    // Check minimum axes requirement based on mode
    size_t min_axes = 5; // Default for "both" mode (needs axes 0 and 4)
    if (joystick_mode_ == "left") {
      min_axes = 2;  // Needs axes 0 and 1
    } else if (joystick_mode_ == "right") {
      min_axes = 5;  // Needs axes 3 and 4
    }
    
    if (msg->axes.size() < min_axes) {
      if (kDebug) printf("Insufficient axes for joystick mode '%s': need %zu, have %zu\n", 
                         joystick_mode_.c_str(), min_axes, msg->axes.size());
      return;
    }
    
    float steer_joystick = 0.0;
    float drive_joystick = 0.0;
    
    // Parse joystick mode configuration
    if (joystick_mode_ == "both") {
      // Default mode: left stick for steering, right stick for drive
      steer_joystick = -msg->axes[0];  // Left stick horizontal
      drive_joystick = -msg->axes[4];  // Right stick vertical
    } else if (joystick_mode_ == "left") {
      // Left stick only: horizontal for steering, vertical for drive
      steer_joystick = -msg->axes[0];  // Left stick horizontal
      drive_joystick = -msg->axes[1];  // Left stick vertical
    } else if (joystick_mode_ == "right") {
      // Right stick only: horizontal for steering, vertical for drive
      steer_joystick = -msg->axes[3];  // Right stick horizontal
      drive_joystick = -msg->axes[4];  // Right stick vertical
    } else {
      // Default to both mode if invalid configuration
      if (kDebug) printf("Invalid joystick_mode '%s', using default 'both'\n", joystick_mode_.c_str());
      steer_joystick = -msg->axes[0];  // Left stick horizontal
      drive_joystick = -msg->axes[4];  // Right stick vertical
    }
    
    const bool turbo_mode = (msg->axes[2] >= 0.9);
    const float max_speed = (turbo_mode ? turbo_speed_ : normal_speed_);
    float speed = drive_joystick * max_speed;
    
    // Apply 2D Parametric Bezier curve to steering input
    // P0=(-1,-1), P1=(-xm,-ym), P2=(0,0), P3=(xm,ym), P4=(1,1)
    // We need to find t such that BezierX(t) = steer_joystick
    // Then steering_output = BezierY(t)
    
    float x_target = steer_joystick;
    
    // Initial guess for t (linear mapping from [-1, 1] to [0, 1])
    float t = (x_target + 1.0f) * 0.5f;
    
    // Newton's method to solve for t
    const int kMaxIter = 10;
    const float kEpsilon = 1e-4f;
    
    for (int i = 0; i < kMaxIter; ++i) {
      float x_val = Bezier4(t, -1.0f, -steering_curve_xm_, 0.0f, steering_curve_xm_, 1.0f);
      float error = x_val - x_target;
      
      if (std::abs(error) < kEpsilon) break;
      
      float dx_dt = Bezier4Prime(t, -1.0f, -steering_curve_xm_, 0.0f, steering_curve_xm_, 1.0f);
      
      // Avoid division by zero
      if (std::abs(dx_dt) < 1e-6f) break;
      
      t -= error / dx_dt;
      
      // Clamp t to [0, 1]
      t = std::max(0.0f, std::min(1.0f, t));
    }
    
    // Compute Y value for the found t
    float steer_curved = Bezier4(t, -1.0f, -steering_curve_ym_, 0.0f, steering_curve_ym_, 1.0f);
    
    float steering_angle = steer_curved * max_steering_angle_;
    mux_drive_speed_ = speed;
    mux_steering_angle_ = steering_angle;
    if (kDebug) printf("Mode: %s, Speed: %7.2f, Steering: %.1f\u00b0\n", 
                       joystick_mode_.c_str(), speed, math_util::RadToDeg(steering_angle));
  }
  
  
  // Instant override logic
  bool override_active = false;
  override_drive_active_ = false;
  override_steer_active_ = false;
  
  if (instant_override_ && (drive_mode_ == kAutonomousDrive || drive_mode_ == kAutonomousContinuousDrive)) {
    // Check for significant joystick input
    static const float kOverrideEps = 0.1; 
    
    // Check drive stick
    float drive_val = 0;
    if (joystick_mode_ == "both" || joystick_mode_ == "right") {
       if (msg->axes.size() > 4) drive_val = msg->axes[4];
    } else {
       if (msg->axes.size() > 1) drive_val = msg->axes[1];
    }
    
    // Check steer stick
    float steer_val = 0;
    if (joystick_mode_ == "both" || joystick_mode_ == "left") {
        if (msg->axes.size() > 0) steer_val = msg->axes[0];
    } else {
        if (msg->axes.size() > 3) steer_val = msg->axes[3];
    }
    
    if (std::abs(drive_val) > kOverrideEps) {
        override_drive_active_ = true;
    }
    if (std::abs(steer_val) > kOverrideEps) {
        override_steer_active_ = true;
    }
    
    if (override_drive_active_ || override_steer_active_) {
        // Calculate joystick commands
        float steer_joystick = 0.0;
        float drive_joystick = 0.0;
        
        if (joystick_mode_ == "both") {
          steer_joystick = -msg->axes[0]; 
          drive_joystick = -msg->axes[4]; 
        } else if (joystick_mode_ == "left") {
          steer_joystick = -msg->axes[0]; 
          drive_joystick = -msg->axes[1]; 
        } else if (joystick_mode_ == "right") {
          steer_joystick = -msg->axes[3]; 
          drive_joystick = -msg->axes[4]; 
        } else {
          steer_joystick = -msg->axes[0]; 
          drive_joystick = -msg->axes[4]; 
        }
        
        const bool turbo_mode = (msg->axes[2] >= 0.9);
        const float max_speed = (turbo_mode ? turbo_speed_ : normal_speed_);
        float speed = drive_joystick * max_speed;
        
        float x_target = steer_joystick;
        float t = (x_target + 1.0f) * 0.5f;
        const int kMaxIter = 10;
        const float kEpsilon = 1e-4f;
        for (int i = 0; i < kMaxIter; ++i) {
          float x_val = Bezier4(t, -1.0f, -steering_curve_xm_, 0.0f, steering_curve_xm_, 1.0f);
          float error = x_val - x_target;
          if (std::abs(error) < kEpsilon) break;
          float dx_dt = Bezier4Prime(t, -1.0f, -steering_curve_xm_, 0.0f, steering_curve_xm_, 1.0f);
          if (std::abs(dx_dt) < 1e-6f) break;
          t -= error / dx_dt;
          t = std::max(0.0f, std::min(1.0f, t));
        }
        float steer_curved = Bezier4(t, -1.0f, -steering_curve_ym_, 0.0f, steering_curve_ym_, 1.0f);
        float steering_angle = steer_curved * max_steering_angle_;
        
        if (override_drive_active_) {
            mux_drive_speed_ = speed;
        }
        if (override_steer_active_) {
            mux_steering_angle_ = steering_angle;
        }
        
        override_active = true;
    }
  }

  // Publish override status
  std_msgs::msg::Bool override_msg;
  override_msg.data = override_active;
  if (override_pub_) {
      override_pub_->publish(override_msg);
  }

  if (drive_mode_ == kAutonomousDrive || 
      drive_mode_ == kAutonomousContinuousDrive) {
    std_msgs::msg::Bool bmsg;
    bmsg.data = true;
    autonomy_enabler_pub_->publish(bmsg);
  } else {
    std_msgs::msg::Bool bmsg;
    bmsg.data = false;
    autonomy_enabler_pub_->publish(bmsg);
  }
}

  /* TODO or TO-THINKABOUT LIST
    - what should we do on startup? send brake or zero command?
    - what to do if the vesc interface gives an error?
    - check version number against know compatable?
    - should we wait until we receive telemetry before sending commands?
    - should we track the last motor command
    - what to do if no motor command received recently?
    - what to do if no servo command received recently?
    - what is the motor safe off state (0 current?)
    - what to do if a command parameter is out of range, ignore?
    - try to predict vesc bounds (from vesc config) and command detect bounds
errors
  */

float Clip(float x, float x_min, float x_max, const char* name) {
  if (x < x_min) {
    fprintf(stderr,
            "Clipping %s value %f to min limit, %f\n",
            name,
            x,
            x_min);
    return x_min;
  }
  if (x > x_max) {
    fprintf(stderr,
            "Clipping %s value %f to max limit, %f\n",
            name,
            x,
            x_max);
    return x_max;
  }
  return x;
}

void VescDriver::sendDriveCommands() {
  static const bool kDebug = false;
  static float last_speed_ = 0;

  using math_util::Bound;
  const float max_accel =
    ((last_speed_ > 0.0) ? max_accel_ : max_decel_);
  const float max_decel =
    ((last_speed_ > 0.0) ? max_decel_ : max_accel_);
  const float smooth_speed = math_util::Clamp<float>(
      mux_drive_speed_,
      last_speed_ - kCommandInterval * max_decel,
      last_speed_ + kCommandInterval * max_accel);
  last_speed_ = smooth_speed;
  last_smooth_speed_ = smooth_speed;  // Store for EKF
  if (kDebug) {
    printf("%7.2f %7.2f %.1f\u00b0\n",
           mux_drive_speed_, smooth_speed, mux_steering_angle_);
  }
  const float erpm =
      speed_to_erpm_gain_ * smooth_speed + speed_offset_trim_;

  // calc steering angle (servo)
  const float servo = steering_to_servo_gain_ * mux_steering_angle_ +
      steering_offset_trim_;

  // Set speed command.
  const float erpm_clipped = Clip(erpm, -erpm_speed_limit_, erpm_speed_limit_, "erpm");
  vesc_.setSpeed(erpm_clipped);

  // Set servo position command.
  const float clipped_servo = Clip(servo, servo_min_, servo_max_, "servo");
  vesc_.setServo(clipped_servo);
  mux_steering_angle_ = (clipped_servo - steering_offset_trim_) 
                        / steering_to_servo_gain_;
  last_steering_angle_ = mux_steering_angle_;

  const float clipped_speed = (erpm_clipped - speed_offset_trim_) / speed_to_erpm_gain_;
  drive_pub_->publish(CalculateDriveCmd(clipped_speed, mux_steering_angle_));
}

void VescDriver::timerCallback() {
  static const double kMaxInitPeriod = 2.0;
  static const double kTStart = rclcpp::Clock(RCL_ROS_TIME).now().seconds();

  if (kDebug) printf("TIMER CALLBACK\n");
  checkCommandTimeout();
  // VESC interface should not unexpectedly disconnect, but test for it anyway
  CHECK(vesc_.isConnected()) 
      << "Unexpectedly disconnected from serial port.";

  /*
   * Driver state machine, modes:
   *  INITIALIZING - request and wait for vesc version
   *  OPERATING - receiving commands from subscriber topics
   */
  if (driver_mode_ == MODE_INITIALIZING) {
  CHECK_LE(rclcpp::Clock(RCL_ROS_TIME).now().seconds() - kTStart, kMaxInitPeriod) 
        << "FAIL: Timed out while trying to initialize VESC.\n";

    if (kDebug) printf("INITIALIZING\n");
    // request version number, return packet will update the internal version
    // numbers
    vesc_.requestFWVersion();
    if (fw_version_major_ >= 0 && fw_version_minor_ >= 0) {
      printf("Connected to VESC with firmware version %d.%d\n",
             fw_version_major_, fw_version_minor_);
      driver_mode_ = MODE_OPERATING;
    }
  } else if (driver_mode_ == MODE_OPERATING) {
    sendDriveCommands();
    if (kDebug) printf("OPERATING\n");
    // poll for vesc state (telemetry)
    vesc_.requestState();
  } else {
    if (kDebug) printf("FAIL: UNKNOWN STATE!\n");
    // unknown mode, how did that happen?
    assert("unknown driver mode");
  }
}

void VescDriver::updateOdometry(float rpm, float tachometer, float steering_angle) {
  static float position_x = 0;
  static float position_y = 0;
  static float orientation = 0; // theta
  static double last_frame_time = rclcpp::Clock(RCL_ROS_TIME).now().seconds();
  static double last_tachometer = tachometer;
  rclcpp::Time current_frame_time = rclcpp::Clock(RCL_ROS_TIME).now();
  double current_frame_time_sec = current_frame_time.seconds();

  // Update the estimated pose
  double del_t = current_frame_time_sec - last_frame_time;
  
  // Tachometer to meters conversion: tachometer counts motor commutations
  // For VESC, typically 6 commutations per electrical revolution
  // ERPM = speed * speed_to_erpm_gain + offset
  // Therefore: tach_to_meters ≈ 60 / (speed_to_erpm_gain * 6)
  static const float tach_to_meters = 60.0f / (speed_to_erpm_gain_ * 6.0f);
  
  float lin_vel = 0;
  float rot_vel = 0;

  // Use EKF fusion if enabled and IMU is available
  if (fuse_imu_ && ekf_ && imu_available_ && mpu6050_) {
    // Enforce monotonically increasing time stamps
    if (del_t >= 0 && del_t < 1.0) {
      // Prediction step with ACTUAL smoothed velocity sent to motor (not raw command)
      ekf_->predict(del_t, last_smooth_speed_, steering_angle);
      
      // Update with wheel odometry from tachometer
      double delta_tach = tachometer - last_tachometer;
      ekf_->updateWheelOdometry(delta_tach, del_t, tach_to_meters, steering_angle);
      
      // Variables for IMU and debug logging (declared here for scope)
      float imu_angular_velocity_x = 0;
      float imu_angular_velocity_y = 0;
      float imu_angular_velocity_z_raw = 0;
      float car_angular_velocity_z = 0;
      
      // Read IMU angular velocity directly from sensor
      try {
        // MPU6050 returns angular velocity in degrees/sec, convert to rad/s
        static const float deg_to_rad = 0.0174533f;
        
        // Read IMU data
        imu_angular_velocity_x = mpu6050_->getAngularVelocityX() * deg_to_rad;
        imu_angular_velocity_y = mpu6050_->getAngularVelocityY() * deg_to_rad;
        imu_angular_velocity_z_raw = mpu6050_->getAngularVelocityZ() * deg_to_rad;
        
        // Apply IMU to car frame transformation
        car_angular_velocity_z = imu_angular_velocity_z_raw;  // Default: no transform
        if (imu_to_car_transform_.size() == 6) {
          // Transform format: [x_sign, x_axis, y_sign, y_axis, z_sign, z_axis]
          float imu_angular_velocity[3] = {imu_angular_velocity_x, imu_angular_velocity_y, imu_angular_velocity_z_raw};
          int z_axis = static_cast<int>(imu_to_car_transform_[5]);
          float z_sign = imu_to_car_transform_[4];
          if (z_axis >= 0 && z_axis < 3) {
            car_angular_velocity_z = z_sign * imu_angular_velocity[z_axis];
          }
        }
        
        // Update step with IMU measurement
        ekf_->updateIMU(car_angular_velocity_z, del_t);
        
        // Publish IMU message
        auto imu_msg = sensor_msgs::msg::Imu();
        imu_msg.header.stamp = current_frame_time;
        imu_msg.header.frame_id = "imu";
        
        // Linear acceleration
        imu_msg.linear_acceleration.x = mpu6050_->getAccelerationX();
        imu_msg.linear_acceleration.y = mpu6050_->getAccelerationY();
        imu_msg.linear_acceleration.z = mpu6050_->getAccelerationZ();
        imu_msg.linear_acceleration_covariance = {0};
        
        // Angular velocity (already transformed to car frame above)
        imu_msg.angular_velocity.x = 0;  // Only using Z axis for now
        imu_msg.angular_velocity.y = 0;
        imu_msg.angular_velocity.z = car_angular_velocity_z;
        imu_msg.angular_velocity_covariance[0] = {0};
        
        // Invalidate quaternion (not calculated)
        imu_msg.orientation_covariance[0] = -1;
        imu_msg.orientation.x = 0;
        imu_msg.orientation.y = 0;
        imu_msg.orientation.z = 0;
        imu_msg.orientation.w = 0;
        
        imu_pub_->publish(imu_msg);
        
        if (kDebug) {
          printf("IMU: angular_velocity_z (car frame) = %.3f rad/s\n", car_angular_velocity_z);
        }
      } catch (const std::exception& e) {
        // If sensor read fails, continue with prediction only
        if (kDebug) {
          printf("Failed to read IMU: %s\n", e.what());
        }
      }
      
      // Get fused state estimate (pose and velocities)
      ekf_->getState(position_x, position_y, orientation, lin_vel, rot_vel);
      
      // Debug logging
      if (debug_ekf_ && debug_log_.is_open()) {
        debug_log_ << std::fixed << std::setprecision(6)
                   << current_frame_time_sec << ","
                   << del_t << ","
                   << last_smooth_speed_ << ","  // Log ACTUAL speed sent to motor
                   << steering_angle << ","
                   << rpm << ","
                   << tachometer << ","
                   << (tachometer - last_tachometer) << ","
                   << tach_to_meters << ","
                   << (imu_angular_velocity_z_raw / 0.0174533f) << ","  // Convert back to deg/s
                   << (car_angular_velocity_z / 0.0174533f) << ","      // Convert back to deg/s
                   << position_x << ","
                   << position_y << ","
                   << orientation << ","
                   << lin_vel << ","
                   << rot_vel << ","
                   << position_x << ","  // odom same as ekf when using ekf
                   << position_y << ","
                   << orientation << ","
                   << lin_vel << ","
                   << rot_vel << "\n";
        debug_log_.flush();
      }
    }
  } else {
    // Standard odometry without EKF fusion
    // Calcuate linear velocity
    lin_vel = (rpm - speed_offset_trim_) / speed_to_erpm_gain_;
    // Clamp velocity to zero for minuscule values - a VESC drift issue.
    if (fabs(lin_vel) < 0.01) {
      lin_vel = 0.0;
    }
    // Calculate angular velocity
    float turn_radius = 0;
    if (steering_angle != 0) {
      turn_radius = wheelbase_ / tan(steering_angle);
      rot_vel = lin_vel / turn_radius;
    }

    // Enforce monotonically increasing time stamps
    if (del_t >= 0) {
      float del_x = lin_vel * del_t * cos(orientation);
      float del_y = lin_vel * del_t * sin(orientation);
      float del_theta = rot_vel * del_t;

      position_x = position_x + del_x;
      position_y = position_y + del_y;
      orientation = math_util::AngleMod(orientation + del_theta);
    }
  }

  // Enforce monotonically increasing time stamps
  if (del_t >= 0) {

    // Create and publish tf2 transform
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = current_frame_time;
    transform.header.frame_id = "odom";
    transform.child_frame_id = "base_link";
    
    transform.transform.translation.x = position_x;
    transform.transform.translation.y = position_y;
    transform.transform.translation.z = 0.0;
    
    transform.transform.rotation.w = cos(0.5 * orientation);
    transform.transform.rotation.x = 0.0;
    transform.transform.rotation.y = 0.0;
    transform.transform.rotation.z = sin(0.5 * orientation);
    
    tf_broadcaster_->sendTransform(transform);
    
    // Create an odometry message
    odom_msg_.header.stamp = current_frame_time;
    odom_msg_.twist.twist.linear.x = lin_vel;
    odom_msg_.twist.twist.angular.z = rot_vel;
    odom_msg_.pose.pose.position.x = position_x;
    odom_msg_.pose.pose.position.y = position_y;
    odom_msg_.pose.pose.orientation.w = cos(0.5 * orientation);
    odom_msg_.pose.pose.orientation.z = sin(0.5 * orientation);
  odom_pub_->publish(odom_msg_);
  } else {
    printf("Odometry messages received out of order.\n") ;
  }
  last_frame_time = current_frame_time_sec;
  last_tachometer = tachometer;
}

void VescDriver::vescPacketCallback(const boost::shared_ptr<VescPacket const>&
packet)
{
  if (packet->name() == "Values") {
    boost::shared_ptr<VescPacketValues const> values =
      boost::dynamic_pointer_cast<VescPacketValues const>(packet);
    // set header stamps from node clock
    {
      uint64_t nsec = nh_->get_clock()->now().nanoseconds();
      state_msg_.header.stamp.sec = static_cast<int32_t>(nsec / 1000000000ULL);
      state_msg_.header.stamp.nanosec = static_cast<uint32_t>(nsec % 1000000000ULL);
    }
    state_msg_.state.voltage_input = values->v_in();
    state_msg_.state.temperature_pcb = values->temp_pcb();
    state_msg_.state.current_motor = values->current_motor();
    state_msg_.state.current_input = values->current_in();
    state_msg_.state.speed = values->rpm();
    state_msg_.state.duty_cycle = values->duty_now();
    state_msg_.state.charge_drawn = values->amp_hours();
    state_msg_.state.charge_regen = values->amp_hours_charged();
    state_msg_.state.energy_drawn = values->watt_hours();
    state_msg_.state.energy_regen = values->watt_hours_charged();
    state_msg_.state.displacement = values->tachometer();
    state_msg_.state.distance_traveled = values->tachometer_abs();
    state_msg_.state.fault_code = values->fault_code();
  state_pub_->publish(state_msg_);

    {
      uint64_t nsec = nh_->get_clock()->now().nanoseconds();
      car_status_msg_.header.stamp.sec = static_cast<int32_t>(nsec / 1000000000ULL);
      car_status_msg_.header.stamp.nanosec = static_cast<uint32_t>(nsec % 1000000000ULL);
    }
    car_status_msg_.battery_voltage = values->v_in();
    car_status_msg_.status = static_cast<uint8_t>(drive_mode_);
  car_status_pub_->publish(car_status_msg_);

    updateOdometry(values->rpm(), values->tachometer(), last_steering_angle_);

  }
  else if (packet->name() == "FWVersion") {
    boost::shared_ptr<VescPacketFWVersion const> fw_version =
      boost::dynamic_pointer_cast<VescPacketFWVersion const>(packet);
    // todo: might need lock here
    fw_version_major_ = fw_version->fwMajor();
    fw_version_minor_ = fw_version->fwMinor();
  } else {
    fprintf(stderr, "Unknown packet type: %s\n", packet->name().c_str());
  }
}

float VescDriver::CalculateSteeringAngle(float lin_vel, float rot_vel) {
  float steering_angle = 0.0;
  if (rot_vel == 0) {
    return steering_angle;
  }

  float turn_radius = lin_vel / rot_vel;
  steering_angle = std::atan(wheelbase_ / turn_radius);
  return steering_angle;
}

void VescDriver::ackermannCurvatureCallback(
    const amrl_msgs::msg::AckermannCurvatureDriveMsg::SharedPtr cmd) {
  t_last_command_ = rclcpp::Clock(RCL_ROS_TIME).now().seconds();
  if (isAutonomous()) {
    if (!override_drive_active_) {
      mux_drive_speed_ = cmd->velocity;
    }

    float effective_speed = mux_drive_speed_;
    
    const float rot_vel = effective_speed * cmd->curvature;
    
    if (!override_steer_active_) {
        mux_steering_angle_ = CalculateSteeringAngle(effective_speed, rot_vel);
    }
  }
}
 

void VescDriver::saveTrimOffsetsToConfig() {
  std::string car_path = FLAGS_config_dir + "/car.lua";
  
  // Read existing content
  std::ifstream file_in(car_path);
  std::string content;
  if (file_in.good()) {
    std::stringstream buffer;
    buffer << file_in.rdbuf();
    content = buffer.str();
    file_in.close();
    
    // Create backup
    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d-%H-%M-%S");
    std::string backup_path = car_path + ".bk-" + ss.str();
    
    std::ofstream backup_out(backup_path);
    if (backup_out.good()) {
      backup_out << content;
      backup_out.close();
      RCLCPP_INFO(nh_->get_logger(), "Created backup config: %s", backup_path.c_str());
    } else {
      RCLCPP_WARN(nh_->get_logger(), "Failed to create backup config: %s", backup_path.c_str());
    }
  }
  
  // Prepare the offset lines with current absolute values
  std::string steering_line = "steering_angle_to_servo_offset = " + 
                              std::to_string(steering_offset_trim_) + ";\n";
  std::string speed_line = "speed_to_erpm_offset = " + 
                          std::to_string(speed_offset_trim_) + ";\n";
  
  // Check if values already exist and update them, or append if not
  // Regex to match existing assignments. Note: values can be negative.
  std::regex steering_regex(R"(steering_angle_to_servo_offset\s*=\s*[-+]?[0-9]*\.?[0-9]+;)");
  std::regex speed_regex(R"(speed_to_erpm_offset\s*=\s*[-+]?[0-9]*\.?[0-9]+;)");
  
  bool has_steering = std::regex_search(content, steering_regex);
  bool has_speed = std::regex_search(content, speed_regex);
  
  if (has_steering) {
    content = std::regex_replace(content, steering_regex, 
                                 "steering_angle_to_servo_offset = " + 
                                 std::to_string(steering_offset_trim_) + ";");
  } else {
    content += steering_line;
  }
  
  if (has_speed) {
    content = std::regex_replace(content, speed_regex,
                                "speed_to_erpm_offset = " + 
                                std::to_string(speed_offset_trim_) + ";");
  } else {
    content += speed_line;
  }
  
  // Write back to file
  std::ofstream file_out(car_path);
  if (file_out.good()) {
    file_out << content;
    file_out.close();
    RCLCPP_INFO(nh_->get_logger(), 
                "Saved offsets to %s: steering=%.3f, speed=%.2f",
                car_path.c_str(), steering_offset_trim_, speed_offset_trim_);
  } else {
    RCLCPP_ERROR(nh_->get_logger(), "Failed to write offsets to %s", car_path.c_str());
  }
}

bool VescDriver::isAutonomous(){
  return drive_mode_ == kAutonomousDrive || drive_mode_ == kAutonomousContinuousDrive;
}
int VescDriver::toggleState(int curToggleState){
  int prevToggleState = prevToggleState_;
  prevToggleState_ = curToggleState;
  if(prevToggleState == 0 && curToggleState == 1){
    return kToggleOn;
  }
  else if(prevToggleState == 1 && curToggleState == 0){
    return kToggleOff;
  }
  else{
    return kNoToggle;
  }
}
} // namespace vesc_driver
