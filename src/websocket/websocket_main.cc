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
#include <QtCore/QCoreApplication>
#include <algorithm>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "rclcpp/rclcpp.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "amrl_msgs/msg/visualization_msg.hpp"
#include "amrl_msgs/msg/localization2_d_msg.hpp"
#include "math/math_util.h"
#include "util/timer.h"
#include "websocket.h"

using amrl_msgs::msg::VisualizationMsg;
using amrl_msgs::msg::Localization2DMsg;
using sensor_msgs::msg::LaserScan;
using std::vector;

DEFINE_double(fps, 10.0, "Max visualization frames rate.");
DEFINE_double(max_age, 2.0, "Maximum age of a message before it gets dropped.");
DECLARE_int32(v);

namespace {
bool run_ = true;
vector<VisualizationMsg> vis_msgs_;
geometry_msgs::msg::PoseWithCovarianceStamped initial_pose_msg_;
geometry_msgs::msg::PoseStamped nav_goal_msg_;
amrl_msgs::msg::Localization2DMsg amrl_initial_pose_msg_;
amrl_msgs::msg::Localization2DMsg amrl_nav_goal_msg_;
Localization2DMsg localization_msg_;
LaserScan laser_scan_;
rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr init_loc_pub_;
rclcpp::Publisher<amrl_msgs::msg::Localization2DMsg>::SharedPtr amrl_init_loc_pub_;
rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr nav_goal_pub_;
rclcpp::Publisher<amrl_msgs::msg::Localization2DMsg>::SharedPtr amrl_nav_goal_pub_;
bool updates_pending_ = false;
RobotWebSocket *server_ = nullptr;
}  // namespace

void LocalizationCallback(const Localization2DMsg& msg) {
  localization_msg_ = msg;
}

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
                   [&msg](const VisualizationMsg &m) {
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
void MergeVector(const std::vector<T> &v1, std::vector<T> *v2) {
  v2->insert(v2->end(), v1.begin(), v1.end());
}

// Merge message m1 into m2.
void MergeMessage(const VisualizationMsg &m1,
                  VisualizationMsg *m2_ptr) {
  VisualizationMsg &m2 = *m2_ptr;
  MergeVector(m1.points, &m2.points);
  MergeVector(m1.lines, &m2.lines);
  MergeVector(m1.arcs, &m2.arcs);
  MergeVector(m1.text_annotations, &m2.text_annotations);
}

void DropOldMessages() {
  const auto now = rclcpp::Clock().now();
  if (rclcpp::Time(now).seconds() - rclcpp::Time(laser_scan_.header.stamp).seconds() > FLAGS_max_age) {
    laser_scan_.header.stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);
  }
  std::remove_if(
      vis_msgs_.begin(),
      vis_msgs_.end(),
      [&now](const VisualizationMsg &m) {
        return (rclcpp::Time(now).seconds() - rclcpp::Time(m.header.stamp).seconds() > FLAGS_max_age);
      });
}

void SendUpdate() {
  if (server_ == nullptr || !updates_pending_) {
    return;
  }
  // DropOldMessages();
  updates_pending_ = false;
  if (laser_scan_.header.stamp.sec == 0 && laser_scan_.header.stamp.nanosec == 0 && vis_msgs_.empty()) {
    return;
  }
  VisualizationMsg local_msgs;
  VisualizationMsg global_msgs;
  for (const VisualizationMsg &m : vis_msgs_) {
    // std::cout << m << std::endl;
    if (m.header.frame_id == "map") {
      MergeMessage(m, &global_msgs);
    } else {
      MergeMessage(m, &local_msgs);
    }
  }
  server_->Send(local_msgs,
                global_msgs,
                laser_scan_,
                localization_msg_);
}

void SetInitialPose(float x, float y, float theta, QString map) {
  if (FLAGS_v > 0) {
    printf("Set initial pose: %s %f,%f, %f\n",
        map.toStdString().c_str(), x, y, math_util::RadToDeg(theta));
  }
  initial_pose_msg_.header.stamp = rclcpp::Clock().now();
  initial_pose_msg_.pose.pose.position.x = x;
  initial_pose_msg_.pose.pose.position.y = y;
  initial_pose_msg_.pose.pose.orientation.w = cos(0.5 * theta);
  initial_pose_msg_.pose.pose.orientation.z = sin(0.5 * theta);
  init_loc_pub_->publish(initial_pose_msg_);
  amrl_initial_pose_msg_.header.stamp = rclcpp::Clock().now();
  amrl_initial_pose_msg_.map = map.toStdString();
  amrl_initial_pose_msg_.pose.x = x;
  amrl_initial_pose_msg_.pose.y = y;
  amrl_initial_pose_msg_.pose.theta = theta;
  amrl_init_loc_pub_->publish(amrl_initial_pose_msg_);
}

void SetNavGoal(float x, float y, float theta, QString map) {
  if (FLAGS_v > 0) {
    printf("Set nav goal: %s %f,%f, %f\n",
        map.toStdString().c_str(), x, y, math_util::RadToDeg(theta));
  }
  nav_goal_msg_.header.stamp = rclcpp::Clock().now();
  nav_goal_msg_.pose.position.x = x;
  nav_goal_msg_.pose.position.y = y;
  nav_goal_msg_.pose.orientation.w = cos(0.5 * theta);
  nav_goal_msg_.pose.orientation.z = sin(0.5 * theta);
  nav_goal_pub_->publish(nav_goal_msg_);
  amrl_nav_goal_msg_.header.stamp = rclcpp::Clock().now();
  amrl_nav_goal_msg_.map = map.toStdString();
  amrl_nav_goal_msg_.pose.x = x;
  amrl_nav_goal_msg_.pose.y = y;
  amrl_nav_goal_msg_.pose.theta = theta;
  amrl_nav_goal_pub_->publish(amrl_nav_goal_msg_);
}

void *RosThread(void *arg) {
  pthread_detach(pthread_self());
  CHECK_NOTNULL(server_);
  QObject::connect(
      server_, &RobotWebSocket::SetInitialPoseSignal, &SetInitialPose);
  QObject::connect(
      server_, &RobotWebSocket::SetNavGoalSignal, &SetNavGoal);

  auto node = rclcpp::Node::make_shared("websocket_node");
  
  auto laser_sub = node->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 5, &LaserCallback);
  auto vis_sub = node->create_subscription<amrl_msgs::msg::VisualizationMsg>(
      "/visualization", 10, &VisualizationCallback);
  auto localization_sub = node->create_subscription<amrl_msgs::msg::Localization2DMsg>(
      "/localization", 10, &LocalizationCallback);
      
  init_loc_pub_ = node->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10);
  nav_goal_pub_ = node->create_publisher<geometry_msgs::msg::PoseStamped>("/move_base_simple/goal", 10);
  amrl_init_loc_pub_ = node->create_publisher<amrl_msgs::msg::Localization2DMsg>("/set_pose", 10);
  amrl_nav_goal_pub_ = node->create_publisher<amrl_msgs::msg::Localization2DMsg>("/set_nav_target", 10);

  RateLoop loop(FLAGS_fps);
  while (rclcpp::ok() && run_) {
    // Consume all pending messages.
    rclcpp::spin_some(node);
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

void InitMessage() {
  laser_scan_.header.stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);
  localization_msg_.header.stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);
  initial_pose_msg_.header.frame_id = "map";
  // Copy RViz's covariance.
  initial_pose_msg_.pose.covariance = {
      0.25, 0, 0, 0, 0, 0,
      0, 0.25, 0, 0, 0, 0,
      0, 0, 0.25, 0, 0, 0,
      0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, math_util::DegToRad(4.0)};
  initial_pose_msg_.pose.pose.position.x = 0;
  initial_pose_msg_.pose.pose.position.y = 0;
  initial_pose_msg_.pose.pose.position.z = 0;
  initial_pose_msg_.pose.pose.orientation.w = 1;
  initial_pose_msg_.pose.pose.orientation.x = 0;
  initial_pose_msg_.pose.pose.orientation.y = 0;
  initial_pose_msg_.pose.pose.orientation.z = 0;
  nav_goal_msg_.header = initial_pose_msg_.header;
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  google::ParseCommandLineFlags(&argc, &argv, false);
  QCoreApplication a(argc, argv);
  server_ = new RobotWebSocket(10272);
  QObject::connect(
      server_, &RobotWebSocket::closed, &a, &QCoreApplication::quit);

  pthread_t ros_thread_id = 0;
  pthread_create(&ros_thread_id, NULL, &RosThread, NULL);

  const int retval = a.exec();
  run_ = false;
  // Waiting for the created thread to terminate
  pthread_join(ros_thread_id, NULL);
  return retval;
}
