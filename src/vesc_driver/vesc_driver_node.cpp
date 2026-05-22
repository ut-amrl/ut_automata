#include <rclcpp/rclcpp.hpp>
#include "gflags/gflags.h"

#include "glog/logging.h"

#include "vesc_driver/vesc_driver.h"

int main(int argc, char** argv)
{
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);  
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("vesc_driver_node");

  vesc_driver::VescDriver vesc_driver(node);

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
