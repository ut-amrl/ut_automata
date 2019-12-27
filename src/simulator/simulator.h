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

#include "f1tenth_course/AckermannCurvatureDriveMsg.h"

#include "shared/util/timer.h"
#include "shared/math/geometry.h"
#include "simulator/vector_map.h"

#ifndef SIMULATOR_H
#define SIMULATOR_H

using namespace std;

class AccelLimits{
  public:
    double max_accel;  // acceleration limit from 0 to max_vel
    double max_deccel; // acceleration limit from max_vel to 0
    double max_vel;    // maximum velocity along dimension

  public:
    void set(double a,double d,double v)
    {max_accel=a; max_deccel=d; max_vel=v;}

    // return new limits with all parameters scaled by <f>
    AccelLimits operator*(double f) const
    {AccelLimits r; r.set(max_accel*f,max_deccel*f,max_vel*f); return(r);}

    // scale all parameters by <f> in-place
    AccelLimits &operator*=(double f);

    // set limits to <al> with all parameters scaled by <f>
    AccelLimits &set(const AccelLimits &al,double f);
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
  Eigen::Vector2f curLoc;

  static const float startAngle;
  float curAngle;

  double tLastCmd;

  static const float DT;
  static const float kMinR;
  geometry_msgs::PoseStamped truePoseMsg;

  f1tenth_course::AckermannCurvatureDriveMsg last_cmd_;

  std::default_random_engine rng_;
  std::normal_distribution<float> laser_noise_;
  std::normal_distribution<float> angular_error_;

private:
  void initVizMarker(visualization_msgs::Marker& vizMarker,
                     string ns,
                     int id,
                     string type,
                     geometry_msgs::PoseStamped p,
                     geometry_msgs::Point32 scale,
                     double duration,
                     std::vector<float> color);
  void initSimulatorVizMarkers();
  void drawMap();
  void InitalLocationCallback(
      const geometry_msgs::PoseWithCovarianceStamped& msg);
  void DriveCallback(const f1tenth_course::AckermannCurvatureDriveMsg& msg);
  void publishOdometry();
  void publishLaser();
  void publishVisualizationMarkers();
  void publishTransform();
  void update();

public:
  Simulator();
  ~Simulator();
  void init(ros::NodeHandle &n);
  void Run();
};
#endif //SIMULATOR_H
