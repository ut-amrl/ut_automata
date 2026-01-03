#include "rclcpp/rclcpp.hpp"
#include "gflags/gflags.h"

#include "glog/logging.h"

#include "vesc_driver/vesc_driver.h"

int main(int argc, char** argv)
{
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  rclcpp::init(argc, argv);
  auto nh = rclcpp::Node::make_shared("vesc_driver_node");
  auto private_nh = nh; // ROS2 generally uses parameter namespaces instead of separate NodeHandles

  vesc_driver::VescDriver vesc_driver(nh, private_nh);

  rclcpp::spin(nh);
  if (rclcpp::ok()) rclcpp::shutdown();
  return 0;
}
