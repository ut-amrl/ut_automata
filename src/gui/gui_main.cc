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

#include <memory>
#include <string>
#include <vector>
#include <signal.h>
#include <QApplication>
#include <QPushButton>
#include <QMetaType>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "amrl_msgs/msg/ackermann_curvature_drive_msg.hpp"
#include "ut_automata/msg/car_status_msg.hpp"
#include "gui_mainwindow.h"
#include "shared/util/timer.h"

using amrl_msgs::msg::AckermannCurvatureDriveMsg;
using ut_automata::msg::CarStatusMsg;

namespace {
ut_automata_gui::MainWindow* main_window_ = nullptr;
rclcpp::Node::SharedPtr node_;
bool run_ = true;
bool lidar_okay_ = false;
bool camera_okay_ = false;
bool vesc_okay_ = false;
float battery_voltage_ = 0.0;
int drive_mode_ = 0;
float throttle_ = 0;
float steering_ = 0;
}  // namespace

void StatusCallback(const CarStatusMsg::SharedPtr msg) {
  if (!run_ || main_window_ == nullptr) return;
  drive_mode_ = msg->status;
  battery_voltage_ = msg->battery_voltage;
  vesc_okay_ = true;
}

void LidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  lidar_okay_  = true;
}

void DriveCallback(const AckermannCurvatureDriveMsg::SharedPtr msg) {
  throttle_ = msg->velocity;
  steering_ = msg->curvature;
}

void* RosThread(void* arg) {
  pthread_detach(pthread_self());

  auto status_sub = node_->create_subscription<CarStatusMsg>(
      "car_status", 1, &StatusCallback);
  auto lidar_sub = node_->create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", 1, &LidarCallback);
  auto drive_sub = node_->create_subscription<AckermannCurvatureDriveMsg>(
      "ackermann_curvature_drive", 1, &DriveCallback);

  RateLoop loop(5.0);
  while(rclcpp::ok() && run_) {
    throttle_ = steering_ = 0;
    vesc_okay_ = lidar_okay_ = camera_okay_ = false;
    rclcpp::spin_some(node_);
    main_window_->UpdateStatus(drive_mode_,
                               battery_voltage_,
                               vesc_okay_,
                               lidar_okay_,
                               camera_okay_,
                               throttle_,
                               steering_);
    loop.Sleep();
  }

  pthread_exit(NULL);
  return nullptr;
}

void SignalHandler(int num) {
  run_ = false;
  if (num == SIGINT) {
    exit(0);
  } else {
    exit(1);
  }
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  node_ = std::make_shared<rclcpp::Node>("ut_automata_gui");
  signal(SIGINT, &SignalHandler);
  qRegisterMetaType<std::vector<std::string> >("std::vector<std::string>");

  QApplication app(argc, argv);
  main_window_ = new ut_automata_gui::MainWindow();
  main_window_->showFullScreen();

  pthread_t ptid = 0;
  pthread_create(&ptid, NULL, &RosThread, NULL);
  const int retval = app.exec();
  run_ = false;
  // Waiting for the created thread to terminate
  pthread_join(ptid, NULL);
  delete main_window_;
  rclcpp::shutdown();
  return retval;
}
