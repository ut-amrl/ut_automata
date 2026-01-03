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
\file    simulator.h
\brief   C++ Interface: Simulator
\author  Joydeep Biswas, (C) 2011
*/
//========================================================================

#include <iostream>
#include <stdio.h>
#include <random>
#include <string>
#include <vector>

#include "eigen3/Eigen/Dense"
#include "geometry_msgs/msg/point32.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include "amrl_msgs/msg/ackermann_curvature_drive_msg.hpp"
#include "amrl_msgs/msg/localization2_d_msg.hpp"

#include "shared/util/random.h"
#include "shared/util/timer.h"
#include "shared/math/geometry.h"
#include "simulator/vector_map.h"

#ifndef SIMULATOR_H
#define SIMULATOR_H

class AccelLimits{
  public:
    double max_accel;  // acceleration limit from 0 to max_vel
    double max_deccel; // acceleration limit from max_vel to 0
    double max_vel;    // maximum velocity along dimension

  public:
    void Set(double a,double d,double v) {
      max_accel = a;
      max_deccel = d;
      max_vel = v;
    }

    // Return new limits with all parameters scaled by <f>.
    AccelLimits operator*(double f) const {
      AccelLimits r;
      r.Set(max_accel * f, max_deccel * f, max_vel * f);
      return(r);
    }

    // Scale all parameters by <f> in-place.
    AccelLimits& operator*=(double f);

    // Set limits to <al> with all parameters scaled by <f>.
    AccelLimits& set(const AccelLimits &al,double f);
};

class Simulator{
  // Forward velocity of the robot at the instantaneous base_link frame.
  double robot_vel_;
  // Angular velocity of the robot.
  double robot_ang_vel_;
  // Last time data was published.
  double last_publish_time_ = 0.0;

  rclcpp::Node::SharedPtr node_;

  rclcpp::Subscription<amrl_msgs::msg::AckermannCurvatureDriveMsg>::SharedPtr drive_subscriber_;
  rclcpp::Subscription<amrl_msgs::msg::Localization2DMsg>::SharedPtr init_subscriber_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr map_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr robot_marker_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr true_pose_publisher_;
  rclcpp::Publisher<amrl_msgs::msg::Localization2DMsg>::SharedPtr localization_publisher_;
  tf2_ros::TransformBroadcaster *tf_broadcaster_;


  sensor_msgs::msg::LaserScan scan_msg_;
  nav_msgs::msg::Odometry odom_msg_;

  vector_map::VectorMap map_;

  visualization_msgs::msg::Marker line_list_marker_;
  visualization_msgs::msg::Marker robot_pos_marker_;

  // True robot location - will be corrupted by actuation error.
  Eigen::Vector2f true_robot_loc_;
  float true_robot_angle_;

  double t_last_cmd_;

  geometry_msgs::msg::PoseStamped truePoseMsg;

  amrl_msgs::msg::AckermannCurvatureDriveMsg last_cmd_;

  amrl_msgs::msg::Localization2DMsg localization_msg_;
  std::string map_name_;

  util_random::Random random_;

  // Odometry-reported robot location - will be according to commands, but
  // starting in arbitrary odometry frame.
  Eigen::Vector2f odom_loc_;
  float odom_angle_;
  bool step_mode_ = false;

private:
  void InitVizMarker(visualization_msgs::msg::Marker& vizMarker,
                     std::string ns,
                     int id,
                     std::string type,
                     geometry_msgs::msg::PoseStamped p,
                     geometry_msgs::msg::Point32 scale,
                     double duration,
                     std::vector<float> color);
  void InitSimulatorVizMarkers();
  void DrawMap();
  void InitalLocationCallback(const amrl_msgs::msg::Localization2DMsg& msg);
  void DriveCallback(const amrl_msgs::msg::AckermannCurvatureDriveMsg& msg);
  void PublishOdometry();
  void PublishLaser();
  void PublishVisualizationMarkers();
  void PublishTransform();
  void Update();

public:
  Simulator();
  ~Simulator();
  void Init(rclcpp::Node::SharedPtr node);
  void ResetState();
  void Run();
  void RunIteration();
  void SetStepMode(bool step_mode);

  void Step(const amrl_msgs::msg::AckermannCurvatureDriveMsg& cmd,
            nav_msgs::msg::Odometry* odom_msg,
            sensor_msgs::msg::LaserScan* scan_msg,
            amrl_msgs::msg::Localization2DMsg* localization_msg);
};
#endif //SIMULATOR_H
