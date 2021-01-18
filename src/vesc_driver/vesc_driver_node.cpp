#include <ros/ros.h>
#include "gflags/gflags.h"

#include "vesc_driver/vesc_driver.h"

int main(int argc, char** argv)
{
  google::ParseCommandLineFlags(&argc, &argv, true);
  
  ros::init(argc, argv, "vesc_driver_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  vesc_driver::VescDriver vesc_driver(nh, private_nh);

  ros::spin();

  return 0;
}
