#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

#include "radio_driver/radio_driver.h"

int main(int argc, char** argv) {
    // Initialize ROS
    ros::init(argc, argv, "radio_driver");
    ros::NodeHandle nh;

    radio_driver::RadioDriver radio_driver(nh);

    // Spin
    ros::spin();
}
