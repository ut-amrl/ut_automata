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
#include <QApplication>
#include <QPushButton>
#include <QMetaType>
#include <memory>
#include <atomic>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "amrl_msgs/msg/ackermann_curvature_drive_msg.hpp"
#include "ut_automata/msg/car_status_msg.hpp"
#include "gui_mainwindow.h"
#include "shared/util/timer.h"

using amrl_msgs::msg::AckermannCurvatureDriveMsg;
using ut_automata::msg::CarStatusMsg;
using sensor_msgs::msg::LaserScan;

namespace {
ut_automata_gui::MainWindow* main_window_ = nullptr;
std::atomic_bool run_{true};
std::atomic_bool lidar_okay_{false};
std::atomic_bool camera_okay_{false};
std::atomic_bool vesc_okay_{false};
std::atomic<float> battery_voltage_{0.0f};
std::atomic_int drive_mode_{0};
std::atomic<float> throttle_{0.0f};
std::atomic<float> steering_{0.0f};
}  // namespace

void StatusCallback(const CarStatusMsg::SharedPtr msg) {
  if (!run_.load() || main_window_ == nullptr) return;
  drive_mode_.store(msg->status);
  battery_voltage_.store(msg->battery_voltage);
  vesc_okay_.store(true);
}

void LidarCallback(const LaserScan::SharedPtr /*msg*/) {
  lidar_okay_.store(true);
}

void DriveCallback(const AckermannCurvatureDriveMsg::SharedPtr msg) {
  throttle_.store(msg->velocity);
  steering_.store(msg->curvature);
}

void* RosThread(void* arg) {
  pthread_detach(pthread_self());

  auto node = rclcpp::Node::make_shared("ut_automata_gui");
  auto status_sub = node->create_subscription<CarStatusMsg>(
      "car_status", 10u, &StatusCallback);
  auto lidar_sub = node->create_subscription<LaserScan>(
      "scan", 10u, &LidarCallback);
  auto drive_sub = node->create_subscription<AckermannCurvatureDriveMsg>(
      "ackermann_curvature_drive", 10u, &DriveCallback);

  RateLoop loop(5.0);
  while (rclcpp::ok() && run_.load()) {
    throttle_.store(0.0f);
    steering_.store(0.0f);
    vesc_okay_.store(false);
    lidar_okay_.store(false);
    camera_okay_.store(false);
    rclcpp::spin_some(node);
    main_window_->UpdateStatus(drive_mode_.load(),
                               battery_voltage_.load(),
                               vesc_okay_.load(),
                               lidar_okay_.load(),
                               camera_okay_.load(),
                               throttle_.load(),
                               steering_.load());
    loop.Sleep();
  }

  pthread_exit(NULL);
  return nullptr;
}

void SignalHandler(int num) {
  run_.store(false);
  // shutdown rclcpp if initialized
  if (rclcpp::ok()) rclcpp::shutdown();
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

  QApplication app(argc, argv);
  main_window_ = new ut_automata_gui::MainWindow();
  main_window_->showFullScreen();

  pthread_t ptid = 0;
  pthread_create(&ptid, NULL, &RosThread, NULL);
  const int retval = app.exec();
  run_.store(false);
  // Waiting for the created thread to terminate
  pthread_join(ptid, NULL);
  delete main_window_;
  if (rclcpp::ok()) rclcpp::shutdown();
  return retval;
}
