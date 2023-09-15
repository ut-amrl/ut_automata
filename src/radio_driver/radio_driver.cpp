#include "radio_driver/radio_driver.h"

static const bool kDebug = false;
static const float kCommandRate = 20;
static const float kCommandInterval = 1.0 / kCommandRate;

namespace radio_driver {

RadioDriver::RadioDriver(ros::NodeHandle nh) {
    // TODO: init radio device

    joystick_sub_ =
        nh.subscribe("/joystick", 10, &RadioDriver::joystickCallback, this);
    timer_ = nh.createSteadyTimer(ros::WallDuration(kCommandInterval),
                                  &RadioDriver::timerCallback, this);
}

void RadioDriver::joystickCallback(const sensor_msgs::Joy& msg) {
    // TODO: track this state
}

void RadioDriver::timerCallback(const ros::SteadyTimerEvent& event) {
    // TODO: publish commands
}

}  // namespace radio_driver
