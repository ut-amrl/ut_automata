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
\brief   C++ Implementation: Simulator
\author  Joydeep Biswas, (C) 2011
*/
//========================================================================

#include <math.h>
#include <stdio.h>

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "f1tenth_course/AckermannCurvatureDriveMsg.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "gflags/gflags.h"

#include "simulator.h"
#include "config_reader/config_reader.h"
#include "shared/math/geometry.h"
#include "shared/math/line2d.h"
#include "shared/math/math_util.h"
#include "shared/ros/ros_helpers.h"
#include "shared/util/timer.h"
#include "vector_map.h"

DEFINE_bool(localize, false, "Publish localization");

using Eigen::Rotation2Df;
using Eigen::Vector2f;
using f1tenth_course::AckermannCurvatureDriveMsg;
using geometry::Heading;
using geometry::line2f;
using geometry_msgs::PoseWithCovarianceStamped;
using math_util::AngleMod;
using math_util::DegToRad;
using math_util::RadToDeg;
using std::atan2;
using vector_map::VectorMap;

CONFIG_STRING(cMapName, "map_name");
CONFIG_FLOAT(cCarLength, "car_length");
CONFIG_FLOAT(cCarWidth, "car_width");
CONFIG_FLOAT(cCarHeight, "car_height");
CONFIG_FLOAT(cRearAxleOffset, "rear_axle_offset");
CONFIG_FLOAT(cLaserLocX, "laser_loc.x");
CONFIG_FLOAT(cLaserLocY, "laser_loc.y");
CONFIG_FLOAT(cLaserLocZ, "laser_loc.z");
CONFIG_FLOAT(cStartX, "start_x");
CONFIG_FLOAT(cStartY, "start_y");
CONFIG_FLOAT(cStartAngle, "start_angle");
CONFIG_FLOAT(cDT, "delta_t");
CONFIG_FLOAT(cMinTurnR, "min_turn_radius");
CONFIG_FLOAT(cMaxAccel, "max_accel");
CONFIG_FLOAT(cMaxSpeed, "max_speed");
config_reader::ConfigReader reader({"config/f1_config.lua"});

Simulator::Simulator() {
  tLastCmd = GetMonotonicTime();
  truePoseMsg.header.seq = 0;
  truePoseMsg.header.frame_id = "map";
}

Simulator::~Simulator() { }

void Simulator::init(ros::NodeHandle& n) {
  scanDataMsg.header.seq = 0;
  scanDataMsg.header.frame_id = "base_laser";
  scanDataMsg.angle_min = DegToRad(-135.0);
  scanDataMsg.angle_max = DegToRad(135.0);
  scanDataMsg.range_min = 0.02;
  scanDataMsg.range_max = 10.0;
  scanDataMsg.angle_increment = DegToRad(0.25);
  scanDataMsg.intensities.clear();
  scanDataMsg.time_increment = 0.0;
  scanDataMsg.scan_time = 0.05;

  odometryTwistMsg.header.seq = 0;
  odometryTwistMsg.header.frame_id = "odom";
  odometryTwistMsg.child_frame_id = "base_footprint";

  curLoc = Vector2f(cStartX, cStartY);
  curAngle = cStartAngle;

  initSimulatorVizMarkers();
  drawMap();

  driveSubscriber = n.subscribe(
      "/ackermann_curvature_drive",
      1,
      &Simulator::DriveCallback,
      this);
  initSubscriber = n.subscribe(
      "/initialpose", 1, &Simulator::InitalLocationCallback, this);
  odometryTwistPublisher = n.advertise<nav_msgs::Odometry>("/odom",1);
  laserPublisher = n.advertise<sensor_msgs::LaserScan>("/scan", 1);
  mapLinesPublisher = n.advertise<visualization_msgs::Marker>(
      "/simulator_visualization", 6);
  posMarkerPublisher = n.advertise<visualization_msgs::Marker>(
      "/simulator_visualization", 6);
  truePosePublisher = n.advertise<geometry_msgs::PoseStamped>(
      "/simulator_true_pose", 1);
  localizationPublisher = n.advertise<geometry_msgs::Pose2D>(
      "/localization", 1);
  br = new tf::TransformBroadcaster();
}

void Simulator::InitalLocationCallback(const PoseWithCovarianceStamped& msg) {
  curLoc = Vector2f(msg.pose.pose.position.x, msg.pose.pose.position.y);
  curAngle = 2.0 *
      atan2(msg.pose.pose.orientation.z, msg.pose.pose.orientation.w);
  printf("Set robot pose: %.2f,%.2f, %.1f\u00b0\n",
         curLoc.x(),
         curLoc.y(),
         RadToDeg(curAngle));
}


/**
 * Helper method that initializes visualization_msgs::Marker parameters
 * @param vizMarker   pointer to the visualization_msgs::Marker object
 * @param ns          namespace for marker (string)
 * @param id          id of marker (int) - must be unique for each marker;
 *                      0, 1, and 2 are already used
 * @param type        specifies type of marker (string); available options:
 *                      arrow (default), cube, sphere, cylinder, linelist,
 *                      linestrip, points
 * @param p           stamped pose to define location and frame of marker
 * @param scale       scale of the marker; see visualization_msgs::Marker
 *                      documentation for details on the parameters
 * @param duration    lifetime of marker in RViz (double); use duration of 0.0
 *                      for infinite lifetime
 * @param color       vector of 4 float values representing color of marker;
 *                    0: red, 1: green, 2: blue, 3: alpha
 */
void Simulator::initVizMarker(visualization_msgs::Marker& vizMarker, string ns,
    int id, string type, geometry_msgs::PoseStamped p,
    geometry_msgs::Point32 scale, double duration, vector<float> color) {

  vizMarker.header.frame_id = p.header.frame_id;
  vizMarker.header.stamp = ros::Time::now();

  vizMarker.ns = ns;
  vizMarker.id = id;

  if (type == "arrow") {
    vizMarker.type = visualization_msgs::Marker::ARROW;
  } else if (type == "cube") {
    vizMarker.type = visualization_msgs::Marker::CUBE;
  } else if (type == "sphere") {
    vizMarker.type = visualization_msgs::Marker::SPHERE;
  } else if (type == "cylinder") {
    vizMarker.type = visualization_msgs::Marker::CYLINDER;
  } else if (type == "linelist") {
    vizMarker.type = visualization_msgs::Marker::LINE_LIST;
  } else if (type == "linestrip") {
    vizMarker.type = visualization_msgs::Marker::LINE_STRIP;
  } else if (type == "points") {
    vizMarker.type = visualization_msgs::Marker::POINTS;
  } else {
    vizMarker.type = visualization_msgs::Marker::ARROW;
  }

  vizMarker.pose = p.pose;
  vizMarker.points.clear();
  vizMarker.scale.x = scale.x;
  vizMarker.scale.y = scale.y;
  vizMarker.scale.z = scale.z;

  vizMarker.lifetime = ros::Duration(duration);

  vizMarker.color.r = color.at(0);
  vizMarker.color.g = color.at(1);
  vizMarker.color.b = color.at(2);
  vizMarker.color.a = color.at(3);

  vizMarker.action = visualization_msgs::Marker::ADD;
}

void Simulator::initSimulatorVizMarkers() {
  geometry_msgs::PoseStamped p;
  geometry_msgs::Point32 scale;
  vector<float> color;
  color.resize(4);

  p.header.frame_id = "/map";

  p.pose.orientation.w = 1.0;
  scale.x = 0.02;
  scale.y = 0.0;
  scale.z = 0.0;
  color[0] = 66.0 / 255.0;
  color[1] = 134.0 / 255.0;
  color[2] = 244.0 / 255.0;
  color[3] = 1.0;
  initVizMarker(lineListMarker, "map_lines", 0, "linelist", p, scale, 0.0,
color);

  p.pose.position.z = 0.5 * cCarHeight;
  scale.x = cCarLength;
  scale.y = cCarWidth;
  scale.z = cCarHeight;
  color[0] = 94.0 / 255.0;
  color[1] = 156.0 / 255.0;
  color[2] = 255.0 / 255.0;
  color[3] = 0.8;
  initVizMarker(robotPosMarker, "robot_position", 1, "cube", p, scale, 0.0,
color);

}

void Simulator::drawMap() {
  ros_helpers::ClearMarker(&lineListMarker);
  for (const line2f& l : map_.lines) {
    ros_helpers::DrawEigen2DLine(l.p0, l.p1, &lineListMarker);
  }
}

void Simulator::publishOdometry() {
  tf::Quaternion robotQ = tf::createQuaternionFromYaw(curAngle);

  odometryTwistMsg.header.stamp = ros::Time::now();
  odometryTwistMsg.pose.pose.position.x = curLoc.x();
  odometryTwistMsg.pose.pose.position.y = curLoc.y();
  odometryTwistMsg.pose.pose.position.z = 0.0;
  odometryTwistMsg.pose.pose.orientation.x = robotQ.x();
  odometryTwistMsg.pose.pose.orientation.y = robotQ.y();
  odometryTwistMsg.pose.pose.orientation.z = robotQ.z();;
  odometryTwistMsg.pose.pose.orientation.w = robotQ.w();
  odometryTwistMsg.twist.twist.angular.x = 0.0;
  odometryTwistMsg.twist.twist.angular.y = 0.0;
  odometryTwistMsg.twist.twist.angular.z = angVel;
  odometryTwistMsg.twist.twist.linear.x = vel * cos(curAngle);
  odometryTwistMsg.twist.twist.linear.y = vel * sin(curAngle);
  odometryTwistMsg.twist.twist.linear.z = 0.0;

  odometryTwistPublisher.publish(odometryTwistMsg);

  robotPosMarker.pose.position.x =
      curLoc.x() - cos(curAngle) * cRearAxleOffset;
  robotPosMarker.pose.position.y =
      curLoc.y() - sin(curAngle) * cRearAxleOffset;
  robotPosMarker.pose.position.z = 0.5 * cCarHeight;
  robotPosMarker.pose.orientation.w = 1.0;
  robotPosMarker.pose.orientation.x = robotQ.x();
  robotPosMarker.pose.orientation.y = robotQ.y();
  robotPosMarker.pose.orientation.z = robotQ.z();
  robotPosMarker.pose.orientation.w = robotQ.w();
}

void Simulator::publishLaser() {
  if (map_.file_name != cMapName) {
    map_.Load(cMapName);
    drawMap();
  }
  scanDataMsg.header.stamp = ros::Time::now();
  const Vector2f laserRobotLoc(cLaserLocX, cLaserLocY);
  const Vector2f laserLoc = curLoc + Rotation2Df(curAngle) * laserRobotLoc;

  const int num_rays = static_cast<int>(
      1.0 + (scanDataMsg.angle_max - scanDataMsg.angle_min) /
      scanDataMsg.angle_increment);
  map_.GetPredictedScan(laserLoc,
                        scanDataMsg.range_min,
                        scanDataMsg.range_max,
                        AngleMod(scanDataMsg.angle_min + curAngle),
                        AngleMod(scanDataMsg.angle_max + curAngle),
                        num_rays,
                        &scanDataMsg.ranges);
  laserPublisher.publish(scanDataMsg);
}

void Simulator::publishTransform() {
  tf::Transform transform;
  tf::Quaternion q;

  transform.setOrigin(tf::Vector3(0.0,0.0,0.0));
  transform.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));
  br->sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/map",
"/odom"));

  transform.setOrigin(tf::Vector3(curLoc.x(), curLoc.y(), 0.0));
  q.setRPY(0.0,0.0,curAngle);
  transform.setRotation(q);
  br->sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/odom",
"/base_footprint"));

  transform.setOrigin(tf::Vector3(0.0 ,0.0, 0.0));
  transform.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));
  br->sendTransform(tf::StampedTransform(transform, ros::Time::now(),
"/base_footprint", "/base_link"));

  transform.setOrigin(tf::Vector3(cLaserLocX, cLaserLocY, cLaserLocZ));
  transform.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1));
  br->sendTransform(tf::StampedTransform(transform, ros::Time::now(),
"/base_link", "/base_laser"));
}

void Simulator::publishVisualizationMarkers() {
  mapLinesPublisher.publish(lineListMarker);
  posMarkerPublisher.publish(robotPosMarker);
}

float AbsBound(float x, float bound) {
  if (x > 0.0 && x > bound) {
    return bound;
  } else if (x < 0.0 && x < -bound) {
    return -bound;
  }
  return x;
}

void Simulator::DriveCallback(const AckermannCurvatureDriveMsg& msg) {
 if (!isfinite(msg.velocity) || !isfinite(msg.curvature)) {
    printf("Ignoring non-finite drive values: %f %f\n",
           msg.velocity,
           msg.curvature);
    return;
  }
  last_cmd_ = msg;
  tLastCmd = GetMonotonicTime();
}

void Simulator::update() {
  static const double kMaxCommandAge = 0.1;
  if (GetMonotonicTime() > tLastCmd + kMaxCommandAge) {
    last_cmd_.velocity = 0;
  }
  // Epsilon curvature corresponding to a very large radius of turning.
  static const float kEpsilonCurvature = 1.0 / 1E3;
  // Commanded speed bounded to motion limit.
  const float desired_vel = AbsBound(last_cmd_.velocity, cMaxSpeed);
  // Maximum magnitude of curvature according to turning limits.
  const float max_curvature = 1.0 / cMinTurnR;
  // Commanded curvature bounded to turning limit.
  const float desired_curvature = AbsBound(last_cmd_.curvature, max_curvature);
  // Indicates if the command is for linear motion.
  const bool linear_motion = (fabs(desired_curvature) < kEpsilonCurvature);

  const float dv_max = cDT * cMaxAccel;
  const float bounded_dv = AbsBound(desired_vel - vel, dv_max);
  vel = vel + bounded_dv;
  const float dist = vel * cDT;

  float dx = 0, dy = 0, dtheta = 0;
  if (linear_motion) {
    dx = dist;
    dy = 0;
    dtheta = 0;
  } else {
    const float r = 1.0 / desired_curvature;
    dtheta = dist * desired_curvature;
    dx = r * sin(dtheta);
    dy = r * (1.0 - cos(dtheta));
  }

  curLoc.x() += dx * cos(curAngle) - dy * sin(curAngle);
  curLoc.y() += dx * sin(curAngle) + dy * cos(curAngle);
  curAngle = AngleMod(curAngle + dtheta);

  truePoseMsg.header.stamp = ros::Time::now();
  truePoseMsg.pose.position.x = curLoc.x();
  truePoseMsg.pose.position.y = curLoc.y();
  truePoseMsg.pose.position.z = 0;
  truePoseMsg.pose.orientation.w = cos(0.5 * curAngle);
  truePoseMsg.pose.orientation.z = sin(0.5 * curAngle);
  truePoseMsg.pose.orientation.x = 0;
  truePoseMsg.pose.orientation.y = 0;
  truePosePublisher.publish(truePoseMsg);

}

void Simulator::Run() {
  // Simulate time-step.
  update();
  //publish odometry and status
  publishOdometry();
  //publish laser rangefinder messages
  publishLaser();
  // publish visualization marker messages
  publishVisualizationMarkers();
  //publish tf
  publishTransform();

  if (FLAGS_localize) {
    geometry_msgs::Pose2D localization_msg;
    localization_msg.x = curLoc.x();
    localization_msg.y = curLoc.y();
    localization_msg.theta = curAngle;
    localizationPublisher.publish(localization_msg);
  }
}
