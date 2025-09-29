//========================================================================
//  This software is free: you can redistribute it and/or modify
//  it under the terms of the GNU Lesser General Public License Version 3,
//  as published by the Free Software Foundation.
//
//  This software is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public License
//  Version 3 in the file COPYING that came with this distribution.
//  If not, see <http://www.gnu.org/licenses/>.
//========================================================================
/*!
\file    gui_main.cc
\brief   GUI for F1/10 car.
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include <pthread.h>

#include <string>
#include <vector>
#include <signal.h>
#include <thread>
#include <chrono>
#include <QApplication>
#include <QPushButton>
#include <QMetaType>
#include <memory>
#include <atomic>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>
#include <QPixmap>

#include "amrl_msgs/msg/ackermann_curvature_drive_msg.hpp"
#include "ut_automata/msg/car_status_msg.hpp"
#include "gui_mainwindow.h"
#include "shared/util/timer.h"

using amrl_msgs::msg::AckermannCurvatureDriveMsg;
using ut_automata::msg::CarStatusMsg;
using sensor_msgs::msg::LaserScan;
using sensor_msgs::msg::Image;
using sensor_msgs::msg::Joy;

namespace {
ut_automata_gui::MainWindow* main_window_ = nullptr;
std::atomic_bool run_{true};
std::atomic_bool lidar_okay_{false};
std::atomic_bool joystick_okay_{false};
std::atomic_bool vesc_okay_{false};
std::atomic<float> battery_voltage_{0.0f};
std::atomic_int drive_mode_{0};
std::atomic<float> throttle_{0.0f};
std::atomic<float> steering_{0.0f};
// Track last joystick message time for timeout detection
std::atomic<std::chrono::steady_clock::time_point> last_joystick_time_{std::chrono::steady_clock::time_point{}};
// ROS node shared pointer for proper cleanup
std::shared_ptr<rclcpp::Node> ros_node_ = nullptr;
}  // namespace

void StatusCallback(const CarStatusMsg::SharedPtr msg) {
  if (!run_.load() || main_window_ == nullptr || !rclcpp::ok()) return;
  drive_mode_.store(msg->status);
  battery_voltage_.store(msg->battery_voltage);
  vesc_okay_.store(true);
}

void LidarCallback(const LaserScan::SharedPtr /*msg*/) {
  if (!run_.load() || !rclcpp::ok()) return;
  lidar_okay_.store(true);
}

void DriveCallback(const AckermannCurvatureDriveMsg::SharedPtr msg) {
  if (!run_.load() || !rclcpp::ok()) return;
  throttle_.store(msg->velocity);
  steering_.store(msg->curvature);
}

void CameraCallback(const Image::SharedPtr msg) {
  if (!run_.load() || main_window_ == nullptr || !rclcpp::ok()) return;
  
  try {
    // Convert ROS image to OpenCV image
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
    cv::Mat image = cv_ptr->image;
    
    // Convert OpenCV image to QPixmap
    QImage qimage(image.data, image.cols, image.rows, image.step, QImage::Format_RGB888);
    QPixmap pixmap = QPixmap::fromImage(qimage.rgbSwapped());
    
    // Update the GUI
    main_window_->UpdateCamera(pixmap);
  } catch (cv_bridge::Exception& e) {
    printf("cv_bridge exception: %s\n", e.what());
  }
}

// Joystick callback - marks joystick as okay when messages are received
void JoystickCallback(const Joy::SharedPtr msg) {
  if (!run_.load() || !rclcpp::ok()) return;

  // Update the last joystick message timestamp
  last_joystick_time_.store(std::chrono::steady_clock::now());
  joystick_okay_.store(true);
}

void* RosThread(void* arg) {
  // Don't detach - we need to properly join the thread
  // pthread_detach(pthread_self());

  ros_node_ = rclcpp::Node::make_shared("ut_automata_gui");
  auto status_sub = ros_node_->create_subscription<CarStatusMsg>(
      "car_status", 10u, &StatusCallback);
  auto lidar_sub = ros_node_->create_subscription<LaserScan>(
      "scan", 10u, &LidarCallback);
  auto drive_sub = ros_node_->create_subscription<AckermannCurvatureDriveMsg>(
      "ackermann_curvature_drive", 10u, &DriveCallback);
  auto camera_sub = ros_node_->create_subscription<Image>(
      "/camera_0/image_raw", 10u, &CameraCallback);
  auto joystick_sub = ros_node_->create_subscription<Joy>(
      "joystick", 10u, &JoystickCallback);

  RateLoop loop(5.0);
  while (rclcpp::ok() && run_.load()) {
    throttle_.store(0.0f);
    steering_.store(0.0f);
    vesc_okay_.store(false);
    lidar_okay_.store(false);
    
    // Check joystick timeout - fail if no message received in the last 1 second
    auto now = std::chrono::steady_clock::now();
    auto last_joystick = last_joystick_time_.load();
    if (last_joystick.time_since_epoch().count() == 0) {
      // No joystick message ever received
      joystick_okay_.store(false);
    } else {
      auto time_since_last_joystick = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_joystick).count();
      if (time_since_last_joystick > 1000) {  // 1 second timeout
        static int timeout_count = 0;
        timeout_count++;
        joystick_okay_.store(false);
      }
      // If we received a message within the timeout, joystick_okay_ is already set to true in the callback
    }
    
    try {
      rclcpp::spin_some(ros_node_);
    } catch (const std::exception& e) {
      // Handle any exceptions during spinning, likely due to shutdown
      printf("Exception during spin_some: %s\n", e.what());
      break;
    }
    
    if (main_window_ != nullptr) {
      main_window_->UpdateStatus(drive_mode_.load(),
                                 battery_voltage_.load(),
                                 vesc_okay_.load(),
                                 lidar_okay_.load(),
                                 joystick_okay_.load(),
                                 throttle_.load(),
                                 steering_.load());
    }
    loop.Sleep();
  }

  // Clean up subscriptions and node before exiting thread
  printf("ROS thread shutting down, cleaning up resources...\n");
  status_sub.reset();
  lidar_sub.reset();
  drive_sub.reset();
  camera_sub.reset();
  joystick_sub.reset();
  ros_node_.reset();
  printf("ROS thread cleanup complete.\n");

  pthread_exit(NULL);
  return nullptr;
}

void SignalHandler(int num) {
  printf("\nReceived signal %d, shutting down gracefully...\n", num);
  run_.store(false);
  
  // Give the ROS thread a moment to clean up
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  
  // Shutdown rclcpp if initialized
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  
  if (num == SIGINT) {
    exit(0);
  } else {
    exit(1);
  }
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  signal(SIGINT, &SignalHandler);
  qRegisterMetaType<std::vector<std::string> >("std::vector<std::string>");
  qRegisterMetaType<QPixmap>("QPixmap");

  QApplication app(argc, argv);
  main_window_ = new ut_automata_gui::MainWindow();
  main_window_->showFullScreen();

  pthread_t ptid = 0;
  pthread_create(&ptid, NULL, &RosThread, NULL);
  
  const int retval = app.exec();
  
  // Signal shutdown and wait for ROS thread to finish
  printf("Application exiting, signaling ROS thread to stop...\n");
  run_.store(false);
  pthread_join(ptid, NULL);
  printf("ROS thread joined successfully.\n");
  
  // Clean up GUI before shutting down ROS
  delete main_window_;
  main_window_ = nullptr;
  printf("Main window deleted.\n");
  
  // Shutdown ROS2 cleanly
  if (rclcpp::ok()) {
    printf("Shutting down rclcpp...\n");
    rclcpp::shutdown();
    printf("rclcpp shutdown complete.\n");
  }
  
  return retval;
}
