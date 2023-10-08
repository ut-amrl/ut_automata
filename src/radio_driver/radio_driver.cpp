#include "radio_driver/radio_driver.h"

#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>

#include "math/math_util.h"

static const bool kDebug = false;

// TODO: move these to config
static const float kCommandRate = 20;
static const float kCommandInterval = 1.0 / kCommandRate;
static const float kCommandTimeout = 0.5;
static const float kFullSpeed = 8.0;  // the fastest the car is capable of going at full throttle
static const float kMaxSpeed = 2.0;   // the fastest we set the caw to be able to go
static const float kSpeedToThrottle = 1.0 / kFullSpeed;  // speed * kSpeedToThrottle = throttle
static const float kMaxAccel = 6.0;
static const float kMaxDecel = 6.0;
static const float kMaxCurvature = 1.5;
static const float kCurvatureToSteering = 1.0 / kMaxCurvature;

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
    // TODO: init radio device

    odom_pub_ = nh.advertise<nav_msgs::Odometry>("odom", 1);
    drive_pub_ = nh.advertise<geometry_msgs::TwistStamped>("radio_drive", 1);
    autonomy_enabler_pub_ = nh.advertise<std_msgs::Bool>("autonomy_enabler", 1);

    joystick_sub_ = nh.subscribe("/joystick", 10, &RadioDriver::joystickCallback, this);
    timer_ = nh.createSteadyTimer(ros::WallDuration(kCommandInterval), &RadioDriver::timerCallback,
                                  this);
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
    joystick_velocity_ = -msg.axes[4] * kMaxSpeed;
    joystick_curvature_ = -msg.axes[0] * kMaxCurvature;

    static std_msgs::Bool autonomy_enabler_msg;
    autonomy_enabler_msg.data =
        drive_mode_ == DriveMode::Autonomous || drive_mode_ == DriveMode::ContinuousAutonomous;
    autonomy_enabler_pub_.publish(autonomy_enabler_msg);
}

void RadioDriver::timerCallback(const ros::SteadyTimerEvent& event) {
    static float throttle, steering;

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

    // TODO: make sure this linearity is real
    throttle = vel * kSpeedToThrottle;
    steering = curv * kCurvatureToSteering;

    // TODO: odometry updates
    radio_interface_.sendControl(throttle, steering);
    drive_pub_.publish(generateTwist(vel, curv));

    lastThrottle_ = throttle;
    lastSteering_ = steering;
    lastSpeed_ = vel;
    lastCurvature_ = curv;
}

float RadioDriver::clampVelocity(float velocity) const {
    const float max_accel = lastSpeed_ > 0 ? kMaxAccel : kMaxDecel;
    const float max_decel = lastSpeed_ > 0 ? kMaxDecel : kMaxAccel;
    float accel_clamp = math_util::Clamp(velocity, -kMaxSpeed, kMaxSpeed);
    return math_util::Clamp(accel_clamp, lastSpeed_ - max_decel * kCommandInterval,
                            lastSpeed_ + max_accel * kCommandInterval);
}

float RadioDriver::clampCurvature(float curvature) const {
    return math_util::Clamp(curvature, -kMaxCurvature, kMaxCurvature);
}

}  // namespace radio_driver
