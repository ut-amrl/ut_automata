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
 * \file    websocket_main.cpp
 * \brief   Main entry point for websocket bridge.
 * \author  Joydeep Biswas, (C) 2019
 */
//========================================================================
#include <QCoreApplication>

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

#include "websocket.h"
#include "f1tenth_course/VisualizationMsg.h"
#include "shared/util/timer.h"

using sensor_msgs::LaserScan;

namespace {
bool run_ = true;
f1tenth_course::VisualizationMsg vis_msg_;
RobotWebSocket* server_ = nullptr;
}  // namespace


void LaserCallback(const LaserScan& msg) {
  if (server_) {
    server_->Send(vis_msg_);
  }
}

void* RosThread(void* arg) {
  pthread_detach(pthread_self());

  ros::NodeHandle n;
  ros::Subscriber status_sub =
      n.subscribe("/scan", 1, &LaserCallback);

  RateLoop loop(5.0);
  while(ros::ok() && run_) {
    ros::spinOnce();
    loop.Sleep();
  }

  pthread_exit(NULL);
  return nullptr;
}

void SignalHandler(int) {
  if (!run_) {
    printf("Force Exit.\n");
    exit(0);
  }
  printf("Exiting.\n");
  run_ = false;
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "websocket", ros::init_options::NoSigintHandler);

  pthread_t ptid = 0;
  pthread_create(&ptid, NULL, &RosThread, NULL);

  QCoreApplication a(argc, argv);

  server_ = new RobotWebSocket(10272);
  QObject::connect(server_, &RobotWebSocket::closed, &a, &QCoreApplication::quit);

  const int retval = a.exec();
  run_ = false;
  // Waiting for the created thread to terminate
  pthread_join(ptid, NULL);
  return retval;
}
