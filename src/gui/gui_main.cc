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
#include <mutex>
#include <errno.h>
#include <time.h>
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
// ROS node and subscriptions for proper cleanup
std::shared_ptr<rclcpp::Node> ros_node_ = nullptr;
rclcpp::Subscription<CarStatusMsg>::SharedPtr status_sub_ = nullptr;
rclcpp::Subscription<LaserScan>::SharedPtr lidar_sub_ = nullptr;
rclcpp::Subscription<AckermannCurvatureDriveMsg>::SharedPtr drive_sub_ = nullptr;
rclcpp::Subscription<Image>::SharedPtr camera_sub_ = nullptr;
rclcpp::Subscription<Joy>::SharedPtr joystick_sub_ = nullptr;
std::mutex cleanup_mutex_;
}  // namespace

void StatusCallback(const CarStatusMsg::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(cleanup_mutex_);
  if (!run_.load() || main_window_ == nullptr || !rclcpp::ok()) return;
  drive_mode_.store(msg->status);
  battery_voltage_.store(msg->battery_voltage);
  vesc_okay_.store(true);
}

void LidarCallback(const LaserScan::SharedPtr /*msg*/) {
  std::lock_guard<std::mutex> lock(cleanup_mutex_);
  if (!run_.load() || !rclcpp::ok()) return;
  lidar_okay_.store(true);
}

void DriveCallback(const AckermannCurvatureDriveMsg::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(cleanup_mutex_);
  if (!run_.load() || !rclcpp::ok()) return;
  throttle_.store(msg->velocity);
  steering_.store(msg->curvature);
}

void CameraCallback(const Image::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(cleanup_mutex_);
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
  std::lock_guard<std::mutex> lock(cleanup_mutex_);
  if (!run_.load() || !rclcpp::ok()) return;

  // Update the last joystick message timestamp
  last_joystick_time_.store(std::chrono::steady_clock::now());
  joystick_okay_.store(true);
}

void* RosThread(void* arg) {
  // Don't detach - we need to properly join the thread
  // pthread_detach(pthread_self());

  ros_node_ = rclcpp::Node::make_shared("ut_automata_gui");
  
  // Store subscriptions as shared pointers for proper cleanup
  status_sub_ = ros_node_->create_subscription<CarStatusMsg>(
      "car_status", 10u, &StatusCallback);
  lidar_sub_ = ros_node_->create_subscription<LaserScan>(
      "scan", 10u, &LidarCallback);
  drive_sub_ = ros_node_->create_subscription<AckermannCurvatureDriveMsg>(
      "ackermann_curvature_drive", 10u, &DriveCallback);
  camera_sub_ = ros_node_->create_subscription<Image>(
      "/camera_0/image_raw", 10u, &CameraCallback);
  joystick_sub_ = ros_node_->create_subscription<Joy>(
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
  
  // Lock to prevent callbacks from running during cleanup
  {
    std::lock_guard<std::mutex> lock(cleanup_mutex_);
    
    // First check if rclcpp is still okay before attempting cleanup
    if (rclcpp::ok() && ros_node_) {
      // Reset subscriptions one by one with error handling
      try {
        if (status_sub_) {
          status_sub_.reset();
          printf("Status subscription cleaned up\n");
        }
        if (lidar_sub_) {
          lidar_sub_.reset();
          printf("Lidar subscription cleaned up\n");
        }
        if (drive_sub_) {
          drive_sub_.reset();
          printf("Drive subscription cleaned up\n");
        }
        if (camera_sub_) {
          camera_sub_.reset();
          printf("Camera subscription cleaned up\n");
        }
        if (joystick_sub_) {
          joystick_sub_.reset();
          printf("Joystick subscription cleaned up\n");
        }
      } catch (const std::exception& e) {
        printf("Exception during subscription cleanup: %s\n", e.what());
      }
      
      // Small delay to ensure all cleanup operations complete
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
      
      // Reset the node after all subscriptions are cleaned up
      try {
        if (ros_node_) {
          ros_node_.reset();
          printf("Node cleaned up\n");
        }
      } catch (const std::exception& e) {
        printf("Exception during node cleanup: %s\n", e.what());
      }
    } else {
      printf("ROS2 context already shut down, skipping subscription cleanup\n");
      // Just reset the pointers without attempting ROS cleanup
      status_sub_.reset();
      lidar_sub_.reset();
      drive_sub_.reset();
      camera_sub_.reset();
      joystick_sub_.reset();
      ros_node_.reset();
    }
  }
  
  printf("ROS thread cleanup complete.\n");

  pthread_exit(NULL);
  return nullptr;
}

void SignalHandler(int num) {
  printf("\nReceived signal %d, shutting down gracefully...\n", num);
  run_.store(false);
  
  // Give the ROS thread time to process the shutdown signal
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  
  // Force exit for signals that require immediate shutdown
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
  
  // Application is closing - initiate shutdown sequence
  printf("Application exiting, initiating clean shutdown...\n");
  run_.store(false);
  
  // Wait for ROS thread to finish with a timeout
  printf("Waiting for ROS thread to complete...\n");
  struct timespec timeout;
  clock_gettime(CLOCK_REALTIME, &timeout);
  timeout.tv_sec += 5; // Increased timeout to 5 seconds
  
  int join_result = pthread_timedjoin_np(ptid, NULL, &timeout);
  if (join_result == ETIMEDOUT) {
    printf("ROS thread join timed out, forcing shutdown...\n");
    pthread_cancel(ptid);
    pthread_join(ptid, NULL); // Wait for cancellation to complete
  } else if (join_result == 0) {
    printf("ROS thread joined successfully.\n");
  } else {
    printf("ROS thread join failed with error: %d\n", join_result);
  }
  
  // Clean up GUI first
  printf("Cleaning up GUI...\n");
  delete main_window_;
  main_window_ = nullptr;
  printf("Main window deleted.\n");
  
  // Add a small delay to ensure all cleanup operations are complete
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  
  // Final ROS2 shutdown - only if still initialized
  if (rclcpp::ok()) {
    printf("Shutting down rclcpp...\n");
    try {
      rclcpp::shutdown();
      printf("rclcpp shutdown complete.\n");
    } catch (const std::exception& e) {
      printf("Exception during rclcpp shutdown: %s\n", e.what());
    }
  } else {
    printf("rclcpp already shut down.\n");
  }
  
  return retval;
}
