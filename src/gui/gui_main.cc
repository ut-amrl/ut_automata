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

#include "ros/ros.h"

#include "f1tenth_course/CarStatusMsg.h"
#include "gui_mainwindow.h"
#include "shared/util/timer.h"

using f1tenth_course::CarStatusMsg;

namespace {
f1tenth_gui::MainWindow* main_window_ = nullptr;
bool run_ = true;
}  // namespace

void StatusCallback(const CarStatusMsg& msg) {
  if (!run_ || main_window_ == nullptr) return;
  main_window_->UpdateStatus(msg.status, msg.battery_voltage);
}

void* RosThread(void* arg) {
  pthread_detach(pthread_self());

  ros::NodeHandle n;
  ros::Subscriber status_sub =
      n.subscribe("car_status", 1, &StatusCallback);

  RateLoop loop(5.0);
  while(ros::ok() && run_) {
    ros::spinOnce();
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
  ros::init(argc, argv, "f1tenth_gui", ros::init_options::NoSigintHandler);
  signal(SIGINT, &SignalHandler);
  qRegisterMetaType<std::vector<std::string> >("std::vector<std::string>");

  QApplication app(argc, argv);
  main_window_ = new f1tenth_gui::MainWindow();
  main_window_->showFullScreen();

  pthread_t ptid = 0;
  pthread_create(&ptid, NULL, &RosThread, NULL);
  const int retval = app.exec();
  run_ = false;
  // Waiting for the created thread to terminate
  pthread_join(ptid, NULL);
  delete main_window_;
  return retval;
}
