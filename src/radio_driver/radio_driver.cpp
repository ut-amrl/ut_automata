#include "radio_driver/radio_driver.h"

#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>

#include "config_reader/config_reader.h"
#include "math/math_util.h"

static const bool kDebug = false;

DEFINE_string(config_dir, "config", "Directory for config files");

CONFIG_STRING(serial_port_, "serial_port");
CONFIG_INT(baud_rate_, "baud_rate");
CONFIG_FLOAT(command_rate_, "command_rate");
CONFIG_FLOAT(command_timeout_, "command_timeout");
CONFIG_FLOAT(full_speed_, "full_speed");
CONFIG_FLOAT(max_speed_, "max_speed");
CONFIG_FLOAT(max_accel_, "max_accel");
CONFIG_FLOAT(max_decel_, "max_decel");
CONFIG_FLOAT(max_curvature_, "max_curvature");

namespace radio_driver {

static geometry_msgs::TwistStamped generateTwist(float velocity, float curvature) {
    geometry_msgs::TwistStamped twist_msg;
    twist_msg.header.stamp = ros::Time::now();
    twist_msg.twist.linear.x = velocity;
    twist_msg.twist.linear.y = 0;
    twist_msg.twist.linear.z = 0;
    twist_msg.twist.angular.x = 0;
    twist_msg.twist.angular.y = 0;
    twist_msg.twist.angular.z = velocity * curvature;
    return twist_msg;
}

RadioDriver::RadioDriver(ros::NodeHandle nh) : drive_mode_(DriveMode::Disabled) {
    config_reader::ConfigReader config({FLAGS_config_dir + "/radio.lua"});

    odom_msg_.header.stamp = ros::Time::now();
    odom_msg_.header.frame_id = "odom";
    odom_msg_.child_frame_id = "base_link";

    odom_msg_.twist.twist.linear.x = 0;
    odom_msg_.twist.twist.linear.y = 0;
    odom_msg_.twist.twist.linear.z = 0;
    odom_msg_.twist.twist.angular.x = 0;
    odom_msg_.twist.twist.angular.y = 0;
    odom_msg_.twist.twist.angular.z = 0;
    odom_msg_.twist.covariance = {0.001, 0.0,       0.0, 0.0,       0.0, 0.0,   0.0, 0.001, 0.0,
                                  0.0,   0.0,       0.0, 0.0,       0.0, 0.001, 0.0, 0.0,   0.0,
                                  0.0,   0.0,       0.0, 1000000.0, 0.0, 0.0,   0.0, 0.0,   0.0,
                                  0.0,   1000000.0, 0.0, 0.0,       0.0, 0.0,   0.0, 0.0,   0.03};

    odom_msg_.pose.pose.position.x = 0;
    odom_msg_.pose.pose.position.y = 0;
    odom_msg_.pose.pose.position.z = 0;
    odom_msg_.pose.covariance = {
        0.001, 0.0, 0.0,       0.0, 0.0,       0.0, 0.0, 0.001, 0.0, 0.0,       0.0, 0.0,
        0.0,   0.0, 1000000.0, 0.0, 0.0,       0.0, 0.0, 0.0,   0.0, 1000000.0, 0.0, 0.0,
        0.0,   0.0, 0.0,       0.0, 1000000.0, 0.0, 0.0, 0.0,   0.0, 0.0,       0.0, 0.03};
    odom_msg_.pose.pose.orientation.w = 1;
    odom_msg_.pose.pose.orientation.x = 0;
    odom_msg_.pose.pose.orientation.y = 0;
    odom_msg_.pose.pose.orientation.z = 0;

    radio_interface_.connect(serial_port_, baud_rate_);

    odom_pub_ = nh.advertise<nav_msgs::Odometry>("odom", 1);
    drive_pub_ = nh.advertise<geometry_msgs::TwistStamped>("radio_drive", 1);
    autonomy_enabler_pub_ = nh.advertise<std_msgs::Bool>("autonomy_enabler", 1);

    joystick_sub_ = nh.subscribe("/joystick", 10, &RadioDriver::joystickCallback, this);
    timer_ = nh.createSteadyTimer(ros::WallDuration(1.0 / command_rate_),
                                  &RadioDriver::timerCallback, this);
}

void RadioDriver::ackermannCurvatureCallback(const amrl_msgs::AckermannCurvatureDriveMsg& msg) {
    ackermann_velocity_ = msg.velocity;
    ackermann_curvature_ = msg.curvature;
}

void RadioDriver::joystickCallback(const sensor_msgs::Joy& msg) {
    // TODO: track this state
    static const size_t kManualDriveButton = 4;
    static const size_t kAutonomousDriveButton = 5;

    if (msg.buttons[kManualDriveButton]) {
        drive_mode_ = DriveMode::Manual;
    } else if (msg.buttons[kAutonomousDriveButton]) {
        drive_mode_ = DriveMode::Autonomous;
    } else {
        drive_mode_ = DriveMode::Disabled;
    }

    // TODO: make this configurable
    // TODO: add turbo
    // TODO: check this is the right range
    joystick_velocity_ = -msg.axes[4] * max_speed_;
    joystick_curvature_ = -msg.axes[0] * max_curvature_;

    static std_msgs::Bool autonomy_enabler_msg;
    autonomy_enabler_msg.data =
        drive_mode_ == DriveMode::Autonomous || drive_mode_ == DriveMode::ContinuousAutonomous;
    autonomy_enabler_pub_.publish(autonomy_enabler_msg);
}

void RadioDriver::timerCallback(const ros::SteadyTimerEvent& event) {
    static float throttle, steering;
    // TODO: command timeout

    if (!radio_interface_.isConnected()) {
        return;
    }

    float vel = 0;
    float curv = 0;
    switch (drive_mode_) {
        case DriveMode::Disabled: {
            vel = 0;
            curv = 0;
            break;
        }
        case DriveMode::Manual: {
            // set throttle and steering based on joystick state
            vel = clampVelocity(joystick_velocity_);
            curv = clampCurvature(joystick_curvature_);
            break;
        }
        case DriveMode::Autonomous: {
            // set throttle and steering based on ackermann commands
            vel = clampVelocity(ackermann_velocity_);
            curv = clampCurvature(ackermann_curvature_);
            break;
        }
        case DriveMode::ContinuousAutonomous: {
            // set throttle and steering based on ackermann commands
            vel = clampVelocity(ackermann_velocity_);
            curv = clampCurvature(ackermann_curvature_);
            break;
        }
    }

    throttle = speedToThrottle(vel);
    steering = curvatureToSteering(curv);

    updateOdometry();
    odom_pub_.publish(odom_msg_);
    radio_interface_.sendControl(throttle, steering);
    drive_pub_.publish(generateTwist(vel, curv));

    lastThrottle_ = throttle;
    lastSteering_ = steering;
    lastSpeed_ = vel;
    lastCurvature_ = curv;
}

float RadioDriver::speedToThrottle(float speed) const {
    // TODO: make sure this linearity is real + make this maybe easier to tune
    return speed / full_speed_;
}

float RadioDriver::curvatureToSteering(float curvature) const {
    // TODO: make sure this linearity is real + make this maybe easier to tune
    return curvature / max_curvature_;
}

float RadioDriver::clampVelocity(float velocity) const {
    const float command_interval = 1.0 / command_rate_;
    const float max_accel = lastSpeed_ > 0 ? max_accel_ : max_decel_;
    const float max_decel = lastSpeed_ > 0 ? max_decel_ : max_accel_;
    float accel_clamp = math_util::Clamp(velocity, -max_speed_, max_speed_);
    return math_util::Clamp(accel_clamp, lastSpeed_ - max_decel * command_interval,
                            lastSpeed_ + max_accel * command_interval);
}

float RadioDriver::clampCurvature(float curvature) const {
    return math_util::Clamp(curvature, -max_curvature_, max_curvature_);
}

void RadioDriver::updateOdometry() {
    static float x = 0;
    static float y = 0;
    static float theta = 0;
    static ros::Time lastTime = ros::Time::now();

    float lin_vel = lastSpeed_;
    if (lin_vel <= 0.01) {
        lin_vel = 0;
    }

    float turn_radius = 0;
    float rot_vel = 0;
    if (std::abs(lastCurvature_) >= 1e-4) {
        turn_radius = 1.0 / lastCurvature_;
        rot_vel = lin_vel / turn_radius;
    }

    double dt = (ros::Time::now() - lastTime).toSec();

    float dx = lin_vel * dt * std::cos(theta);
    float dy = lin_vel * dt * std::sin(theta);
    float dtheta = rot_vel * dt;

    x += dx;
    y += dy;
    theta = math_util::AngleMod(theta + dtheta);

    odom_msg_.header.stamp = ros::Time::now();
    odom_msg_.pose.pose.position.x = x;
    odom_msg_.pose.pose.position.y = y;
    odom_msg_.pose.pose.orientation.w = std::cos(theta / 2);
    odom_msg_.pose.pose.orientation.z = std::sin(theta / 2);
    odom_msg_.twist.twist.linear.x = lin_vel;
    odom_msg_.twist.twist.angular.z = rot_vel;

    lastTime = ros::Time::now();
}

}  // namespace radio_driver
