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
#include <algorithm>
#include <QCoreApplication>
#include <vector>

#include "gflags/gflags.h"
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

#include "websocket.h"
#include "f1tenth_course/VisualizationMsg.h"
#include "shared/util/timer.h"

using f1tenth_course::VisualizationMsg;
using sensor_msgs::LaserScan;
using std::vector;

DEFINE_double(max_age, 2.0, "Maximum age of a message before it gets dropped.");

namespace {
bool run_ = true;
vector<VisualizationMsg> vis_msgs_;
LaserScan laser_scan_;
bool updates_pending_ = false;
RobotWebSocket* server_ = nullptr;
}  // namespace


void LaserCallback(const LaserScan& msg) {
  laser_scan_ = msg;
  updates_pending_ = true;
}

void VisualizationCallback(const VisualizationMsg& msg) {
  static bool warning_showed_ = false;
  if (msg.header.frame_id != "base_link" &&
      msg.header.frame_id != "map") {
    if (!warning_showed_) {
      fprintf(stderr, 
              "WARNING: Ignoring visualization for unknown frame '%s'."
              " This message prints only once.\n",
              msg.header.frame_id.c_str());
      warning_showed_ = true;
    }
    return;
  }
  auto prev_msg = 
      std::find_if(vis_msgs_.begin(), 
                   vis_msgs_.end(),
                   [&msg](const VisualizationMsg& m) {
                     return m.ns == msg.ns;
                   });
  if (prev_msg == vis_msgs_.end()) {
    vis_msgs_.push_back(msg);
  } else {
    *prev_msg = msg;
  }
  updates_pending_ = true;
}

template <typename T>
void MergeVector(const std::vector<T>& v1, std::vector<T>* v2) {
  v2->insert(v2->end(), v1.begin(), v1.end());
}

// Merge message m1 into m2.
void MergeMessage(const VisualizationMsg& m1,
                  VisualizationMsg* m2_ptr) {
  VisualizationMsg& m2 = *m2_ptr;
  MergeVector(m1.particles, &m2.particles);
  MergeVector(m1.path_options, &m2.path_options);
  MergeVector(m1.points, &m2.points);
  MergeVector(m1.lines, &m2.lines);
  MergeVector(m1.arcs, &m2.arcs);
}

void DropOldMessages() {
  const auto now = ros::Time::now();
  if ((now - laser_scan_.header.stamp).toSec() > FLAGS_max_age) {
    laser_scan_.header.stamp = ros::Time(0);
  }
  std::remove_if(
      vis_msgs_.begin(),
      vis_msgs_.end(),
      [&now](const VisualizationMsg& m) { 
        return ((now - m.header.stamp).toSec() > FLAGS_max_age);
      });
}

void SendUpdate() {
  if (server_ == nullptr || !updates_pending_) return;
  DropOldMessages();
  updates_pending_ = false;
  if (laser_scan_.header.stamp.toSec() == 0 && vis_msgs_.empty()) {
    return;
  }
  VisualizationMsg local_msgs;
  VisualizationMsg global_msgs;
  for (const VisualizationMsg& m : vis_msgs_) {
    if (m.header.frame_id == "map") {
      MergeMessage(m, &global_msgs);
    } else {
      MergeMessage(m, &local_msgs);
    }
  }
  server_->Send(local_msgs, global_msgs, laser_scan_);
}

void* RosThread(void* arg) {
  pthread_detach(pthread_self());

  ros::NodeHandle n;
  ros::Subscriber laser_sub =
      n.subscribe("/scan", 5, &LaserCallback);
  ros::Subscriber vis_sub =
      n.subscribe("/visualization", 10, &VisualizationCallback);

  RateLoop loop(10.0);
  while(ros::ok() && run_) {
    // Consume all pending messages.
    ros::spinOnce();
    // Update rate is throttled by the rate loop timer.
    SendUpdate();
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
  laser_scan_.header.stamp = ros::Time(0);
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
