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
#include "geometry_msgs/Point32.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "nav_msgs/Odometry.h"
#include "ros/ros.h"
#include "ros/package.h"
#include "sensor_msgs/LaserScan.h"
#include "tf/transform_broadcaster.h"
#include "tf/transform_datatypes.h"
#include "visualization_msgs/Marker.h"

#include "amrl_msgs/AckermannCurvatureDriveMsg.h"
#include "amrl_msgs/Localization2DMsg.h"

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
  Eigen::Vector2f loc;
  double vel;
  double angVel;

  ros::Subscriber driveSubscriber;
  ros::Subscriber initSubscriber;

  ros::Publisher odometryTwistPublisher;
  ros::Publisher laserPublisher;
  ros::Publisher mapLinesPublisher;
  ros::Publisher posMarkerPublisher;
  ros::Publisher truePosePublisher;
  ros::Publisher localizationPublisher;
  tf::TransformBroadcaster *br;


  sensor_msgs::LaserScan scanDataMsg;
  nav_msgs::Odometry odometryTwistMsg;

  vector_map::VectorMap map_;

  visualization_msgs::Marker lineListMarker;
  visualization_msgs::Marker robotPosMarker;

  static const float startX;
  static const float startY;
  static const float startAngle;

  // True robot location - will be corrupted by actuation error.
  Eigen::Vector2f true_robot_loc_;
  float true_robot_angle_;

  double tLastCmd;

  static const float DT;
  static const float kMinR;
  geometry_msgs::PoseStamped truePoseMsg;

  amrl_msgs::AckermannCurvatureDriveMsg last_cmd_;

  amrl_msgs::Localization2DMsg localization_msg_;
  std::string map_name_;

  util_random::Random random_;

  // Odometry-reported robot location - will be according to commands, but
  // starting in arbitrary odometry frame.
  Eigen::Vector2f odom_loc_;
  float odom_angle_;

private:
  void InitVizMarker(visualization_msgs::Marker& vizMarker,
                     std::string ns,
                     int id,
                     std::string type,
                     geometry_msgs::PoseStamped p,
                     geometry_msgs::Point32 scale,
                     double duration,
                     std::vector<float> color);
  void InitSimulatorVizMarkers();
  void DrawMap();
  void InitalLocationCallback(const amrl_msgs::Localization2DMsg& msg);
  void DriveCallback(const amrl_msgs::AckermannCurvatureDriveMsg& msg);
  void PublishOdometry();
  void PublishLaser();
  void PublishVisualizationMarkers();
  void PublishTransform();
  void Update();

public:
  Simulator();
  ~Simulator();
  void Init(ros::NodeHandle &n);
  void Run();
};
#endif //SIMULATOR_H
