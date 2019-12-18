// -*- mode:c++; fill-column: 100; -*-

#include "vesc_driver.h"

#include <atomic>
#include <cassert>
#include <cmath>
#include <sstream>

#include "boost/bind.hpp"
#include "f1tenth_course/VescStateStamped.h"
#include "f1tenth_course/AckermannCurvatureDriveMsg.h"
#include "ackermann_msgs/AckermannDriveStamped.h"
#include "nav_msgs/Odometry.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "shared/math/math_util.h"

static const bool kDebug = false;
static const float kCommandRate = 50;
static const float kCommandInterval = 1.0 / kCommandRate;

using f1tenth_course::VescStateStamped;

namespace {

float mux_drive_speed_ = 0;
float mux_steering_angle_ = 0;

template <typename T>
inline bool getRequiredParam(const ros::NodeHandle& nh,
                             std::string name,
                             T& value) {
  if (nh.getParam(name, value)) return true;
  ROS_FATAL("AckermannToVesc: Parameter %s is required.", name.c_str());
  return false;
}

}  // namespace

namespace vesc_driver
{

VescDriver::VescDriver(ros::NodeHandle nh,
                       ros::NodeHandle private_nh) :
    vesc_(std::string(),
          boost::bind(&VescDriver::vescPacketCallback, this, _1),
          boost::bind(&VescDriver::vescErrorCallback, this, _1)),
    duty_cycle_limit_(private_nh, "duty_cycle", -1.0, 1.0),
    current_limit_(private_nh, "current"),
    brake_limit_(private_nh, "brake"),
    speed_limit_(private_nh, "speed"),
    position_limit_(private_nh, "position"),
    servo_limit_(private_nh, "servo", 0.0, 1.0),
    driver_mode_(MODE_INITIALIZING),
    drive_mode_(kStoppedDrive),
    fw_version_major_(-1),
    fw_version_minor_(-1),
    t_last_command_(0),
    t_last_joystick_(0),
    last_speed_command_(0) {
  odom_msg_.header.stamp = ros::Time::now();
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

  // get vesc serial port address
  std::string port;
  if (!private_nh.getParam("port", port)) {
    ROS_FATAL("VESC communication port parameter required.");
    ros::shutdown();
    return;
  }

  // Load conversion parameters.
  if (!getRequiredParam(private_nh,
                        "speed_to_erpm_gain",
                        speed_to_erpm_gain_) ||
      !getRequiredParam(private_nh,
                        "speed_to_erpm_offset",
                        speed_to_erpm_offset_) ||
      !getRequiredParam(private_nh,
                        "steering_angle_to_servo_gain",
                        steering_to_servo_gain_) ||
      !getRequiredParam(private_nh,
                        "steering_angle_to_servo_offset",
                        steering_to_servo_offset_) ||
      !getRequiredParam(private_nh,
                        "wheelbase",
                        wheel_base_)) {
    ros::shutdown();
    return;
  }

  // attempt to connect to the serial port
  try {
    if (kDebug) printf("CONNECT\n");
    vesc_.connect(port);
  } catch (SerialException e) {
    ROS_FATAL("Failed to connect to the VESC, %s.", e.what());
    ros::shutdown();
    return;
  }
  if (kDebug) printf("CONNECTED\n");
  // create vesc state (telemetry) publisher
  state_pub_ = nh.advertise<VescStateStamped>("sensors/core", 10);

  odom_pub_ = nh.advertise<nav_msgs::Odometry>("odom", 10);

  // since vesc state does not include the servo position, publish the commanded
  // servo position as a "sensor"
  servo_sensor_pub_ =
      nh.advertise<std_msgs::Float64>("sensors/servo_position_command", 10);

  // create ackermann subscriber.
  ackermann_sub_ = nh.subscribe(
      "commands/ackermann", 10, &VescDriver::ackermannCmdCallback, this);
  ackermann_curvature_sub_ = nh.subscribe(
      "/ackermann_curvature_drive",
      10,
      &VescDriver::ackermannCurvatureCallback,
      this);
  joystick_sub_ = nh.subscribe(
      "joystick", 10, &VescDriver::joystickCallback, this);

  if (kDebug) printf("TIMER START\n");
  // create a 50Hz timer, used for state machine & polling VESC telemetry
  timer_ = nh.createSteadyTimer(ros::WallDuration(kCommandInterval),
                                &VescDriver::timerCallback,
                                this);
  if (kDebug) printf("DONE INIT\n");
}

void VescDriver::checkCommandTimeout() {
  static const double kTimeout = 0.5;
  const double t_now = ros::WallTime::now().toSec();
  if (t_now > t_last_command_ + kTimeout ||
      t_now > t_last_joystick_ + kTimeout) {
    mux_drive_speed_ = 0;
    mux_steering_angle_ = 0;
  }
}

void VescDriver::joystickCallback(const sensor_msgs::Joy& msg) {
  static const float kMaxTurnRate = 0.25;
  static const float kTurboSpeed = 2.0;
  static const float kNormalSpeed = 1.0;
  static const size_t kManualDriveButton = 4;
  static const size_t kAutonomousDriveButton = 5;

  if (msg.buttons.size() < 6) return;
  if (msg.buttons[kManualDriveButton] == 1) {
    drive_mode_ = kJoystickDrive;
  } else if (msg.buttons[kAutonomousDriveButton] == 1) {
    drive_mode_ = kAutonomousDrive;
  } else {
    drive_mode_ = kStoppedDrive;
  }
  t_last_joystick_ = ros::WallTime::now().toSec();
  if (msg.axes.size() < 5) return;
  const float steer_joystick = -msg.axes[0];
  const float drive_joystick = -msg.axes[4];
  const bool turbo_mode = (msg.axes[2] >= 0.9);
  const float max_speed = (turbo_mode ? kTurboSpeed : kNormalSpeed);
  float speed = drive_joystick * max_speed;
  float steering_angle = steer_joystick * kMaxTurnRate;

  printf("%.2f %.1f\u00b0\n", speed, math_util::RadToDeg(steering_angle));
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

void VescDriver::sendDriveCommands() {
  static const bool kDebug = true;
  static const float kMaxAcceleration = 4.0; // m/s^2
  static const float kMaxDeceleration = 6.0; // m/s^2
  static float last_speed_ = 0;

  using math_util::Bound;
  const float max_accel =
    ((last_speed_ > 0.0) ? kMaxAcceleration : kMaxDeceleration);
  const float max_decel =
    ((last_speed_ > 0.0) ? kMaxDeceleration : kMaxAcceleration);
  const float smooth_speed = Bound<float>(
      last_speed_ - kCommandInterval * max_decel,
      last_speed_ + kCommandInterval * max_accel,
      mux_drive_speed_);
  last_speed_ = smooth_speed;
  if (kDebug) {
    printf("%.2f %.1f\u00b0\n", smooth_speed, mux_steering_angle_);
  }
  const double erpm =
      speed_to_erpm_gain_ * smooth_speed + speed_to_erpm_offset_;

  // calc steering angle (servo)
  const double servo = steering_to_servo_gain_ * mux_steering_angle_ +
      steering_to_servo_offset_;

  // Set speed command.
  if (false) {
    vesc_.setSpeed(speed_limit_.clip(erpm));
  } else {
    vesc_.setSpeed(0);
  }

  // Set servo position command.
  vesc_.setServo(servo_limit_.clip(servo));
  last_steering_angle_ = mux_steering_angle_;
}

void VescDriver::timerCallback(const ros::SteadyTimerEvent& event) {
  static const double kMaxInitPeriod = 2.0;
  static const double kTStart = ros::WallTime::now().toSec();

  if (kDebug) printf("TIMER CALLBACK\n");
  checkCommandTimeout();
  // VESC interface should not unexpectedly disconnect, but test for it anyway
  if (!vesc_.isConnected()) {
    ROS_FATAL("Unexpectedly disconnected from serial port.");
    timer_.stop();
    ros::shutdown();
    return;
  }

  /*
   * Driver state machine, modes:
   *  INITIALIZING - request and wait for vesc version
   *  OPERATING - receiving commands from subscriber topics
   */
  if (driver_mode_ == MODE_INITIALIZING) {
    if (ros::WallTime::now().toSec() > kTStart + kMaxInitPeriod) {
      ROS_FATAL("FAIL: Timed out while trying to initialize VESC.");
      ros::shutdown();
      return;
    }
    if (kDebug) printf("INITIALIZING\n");
    // request version number, return packet will update the internal version
numbers
    vesc_.requestFWVersion();
    if (fw_version_major_ >= 0 && fw_version_minor_ >= 0) {
      ROS_INFO("Connected to VESC with firmware version %d.%d",
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

void VescDriver::updateOdometry(double rpm, double steering_angle) {
  // TODO: calculate speed based on tachometer as opposed to rpm

  // Calcuate linear velocity
  double lin_vel = (rpm - speed_to_erpm_offset_) / speed_to_erpm_gain_;

  // Calculate angular velocity
  double turn_radius = 0;
  double rot_vel = 0;
  if (steering_angle != 0) {
    turn_radius = wheel_base_ / tan(steering_angle);
    rot_vel = lin_vel / turn_radius;
  }

  static float position_x = 0;
  static float position_y = 0;
  static float orientation = 0; // theta
  static double last_frame_time = ros::Time::now().toSec();
  ros::Time current_frame_time_ros = ros::Time::now();
  double current_frame_time = current_frame_time_ros.toSec();

  // Update the estimated pose
  double del_t = current_frame_time - last_frame_time;

  // Enforce monotonically increasing time stamps
  if (del_t >= 0) {
    float del_x = lin_vel * del_t * cos(orientation);
    float del_y = lin_vel * del_t * sin(orientation);
    float del_theta = rot_vel * del_t;

    position_x = position_x + del_x;
    position_y = position_y + del_y;
    orientation = math_util::AngleMod(orientation + del_theta);

    // Create an odometry message
    odom_msg_.twist.twist.linear.x = lin_vel;
    odom_msg_.twist.twist.angular.z = rot_vel;
    odom_msg_.pose.pose.position.x = position_x;
    odom_msg_.pose.pose.position.y = position_y;
    odom_msg_.pose.pose.orientation.w = cos(0.5 * orientation);
    odom_msg_.pose.pose.orientation.z = sin(0.5 * orientation);
    odom_pub_.publish(odom_msg_);
  } else {
    ROS_INFO("Odometry messages received out of order.") ;
  }
  last_frame_time = current_frame_time;
}

void VescDriver::vescPacketCallback(const boost::shared_ptr<VescPacket const>&
packet)
{
  if (packet->name() == "Values") {
    boost::shared_ptr<VescPacketValues const> values =
      boost::dynamic_pointer_cast<VescPacketValues const>(packet);

    VescStateStamped::Ptr state_msg(new VescStateStamped);
    state_msg->header.stamp = ros::Time::now();
    state_msg->state.voltage_input = values->v_in();
    state_msg->state.temperature_pcb = values->temp_pcb();
    state_msg->state.current_motor = values->current_motor();
    state_msg->state.current_input = values->current_in();
    state_msg->state.speed = values->rpm();
    state_msg->state.duty_cycle = values->duty_now();
    state_msg->state.charge_drawn = values->amp_hours();
    state_msg->state.charge_regen = values->amp_hours_charged();
    state_msg->state.energy_drawn = values->watt_hours();
    state_msg->state.energy_regen = values->watt_hours_charged();
    state_msg->state.displacement = values->tachometer();
    state_msg->state.distance_traveled = values->tachometer_abs();
    state_msg->state.fault_code = values->fault_code();

    state_pub_.publish(state_msg);
    updateOdometry(values->rpm(), last_steering_angle_);

  }
  else if (packet->name() == "FWVersion") {
    boost::shared_ptr<VescPacketFWVersion const> fw_version =
      boost::dynamic_pointer_cast<VescPacketFWVersion const>(packet);
    // todo: might need lock here
    fw_version_major_ = fw_version->fwMajor();
    fw_version_minor_ = fw_version->fwMinor();
  }
}

void VescDriver::vescErrorCallback(const std::string& error) {
  ROS_ERROR("%s", error.c_str());
}

float VescDriver::CalculateSteeringAngle(float lin_vel, float rot_vel) {
  float steering_angle = 0.0;
  if (rot_vel == 0) {
    return steering_angle;
  }

  float turn_radius = lin_vel / rot_vel;
  steering_angle = std::atan(wheel_base_ / turn_radius);
  return steering_angle;
}

void VescDriver::ackermannCurvatureCallback(
    const f1tenth_course::AckermannCurvatureDriveMsg& cmd) {
  t_last_command_ = ros::WallTime::now().toSec();
  mux_drive_speed_ = cmd.velocity;
  const float rot_vel = cmd.velocity * cmd.curvature;
  mux_steering_angle_ = CalculateSteeringAngle(mux_drive_speed_, rot_vel);
}

VescDriver::CommandLimit::CommandLimit(const ros::NodeHandle& nh, const std::string& str,
                                       const boost::optional<double>& min_lower,
                                       const boost::optional<double>& max_upper) :
  name(str)
{
  // check if user's minimum value is outside of the range min_lower to max_upper
  double param_min;
  if (nh.getParam(name + "_min", param_min)) {
    if (min_lower && param_min < *min_lower) {
      lower = *min_lower;
      ROS_WARN_STREAM("Parameter " << name << "_min (" << param_min <<
                      ") is less than the feasible minimum (" << *min_lower << ").");
    }
    else if (max_upper && param_min > *max_upper) {
      lower = *max_upper;
      ROS_WARN_STREAM("Parameter " << name << "_min (" << param_min <<
                      ") is greater than the feasible maximum (" << *max_upper << ").");
    }
    else {
      lower = param_min;
    }
  }
  else if (min_lower) {
    lower = *min_lower;
  }

  // check if the uers' maximum value is outside of the range min_lower to max_upper
  double param_max;
  if (nh.getParam(name + "_max", param_max)) {
    if (min_lower && param_max < *min_lower) {
      upper = *min_lower;
      ROS_WARN_STREAM("Parameter " << name << "_max (" << param_max <<
                      ") is less than the feasible minimum (" << *min_lower << ").");
    }
    else if (max_upper && param_max > *max_upper) {
      upper = *max_upper;
      ROS_WARN_STREAM("Parameter " << name << "_max (" << param_max <<
                      ") is greater than the feasible maximum (" << *max_upper << ").");
    }
    else {
      upper = param_max;
    }
  }
  else if (max_upper) {
    upper = *max_upper;
  }

  // check for min > max
  if (upper && lower && *lower > *upper) {
    ROS_WARN_STREAM("Parameter " << name << "_max (" << *upper
                    << ") is less than parameter " << name << "_min (" << *lower << ").");
    double temp(*lower);
    lower = *upper;
    upper = temp;
  }

  std::ostringstream oss;
  oss << "  " << name << " limit: ";
  if (lower) oss << *lower << " "; else oss << "(none) ";
  if (upper) oss << *upper; else oss << "(none)";
  ROS_DEBUG_STREAM(oss.str());
}

double VescDriver::CommandLimit::clip(double value)
{
  if (lower && value < lower) {
    ROS_INFO_THROTTLE(10, "%s command value (%f) below minimum limit (%f), clipping.",
                      name.c_str(), value, *lower);
    return *lower;
  }
  if (upper && value > upper) {
    ROS_INFO_THROTTLE(10, "%s command value (%f) above maximum limit (%f), clipping.",
                      name.c_str(), value, *upper);
    return *upper;
  }
  return value;
}


} // namespace vesc_driver
