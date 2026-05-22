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

#include <algorithm>
#include <functional>
#include <string>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "builtin_interfaces/msg/duration.hpp"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "amrl_msgs/msg/ackermann_curvature_drive_msg.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "tf2/LinearMath/Quaternion.h"

#include "simulator.h"
#include "amrl_msgs/msg/localization2_d_msg.hpp"
#include "config_reader/config_reader.h"
#include "shared/math/geometry.h"
#include "shared/math/line2d.h"
#include "shared/math/math_util.h"
#include "shared/ros/ros_helpers.h"
#include "shared/util/random.h"
#include "shared/util/timer.h"
#include "vector_map.h"

using Eigen::Rotation2Df;
using Eigen::Vector2f;
using amrl_msgs::msg::AckermannCurvatureDriveMsg;
using geometry::Heading;
using geometry::Line2f;
using geometry_msgs::msg::PoseWithCovarianceStamped;
using math_util::AngleMod;
using math_util::DegToRad;
using math_util::RadToDeg;
using std::atan2;
using std::max;
using std::string;
using std::vector;
using vector_map::VectorMap;

DEFINE_bool(localize, false, "Publish localization");

const string kAmrlMapsDir = ament_index_cpp::get_package_share_directory("amrl_maps");
const string kUtAutomataDir = ament_index_cpp::get_package_share_directory("ut_automata");

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
CONFIG_FLOAT(cPublishRate, "publish_rate");
CONFIG_FLOAT(cSubSampleRate, "sub_sample_rate");
CONFIG_FLOAT(cMinTurnR, "min_turn_radius");
CONFIG_FLOAT(cMaxAccel, "max_accel");
CONFIG_FLOAT(cMaxSpeed, "max_speed");
CONFIG_FLOAT(cLaserStdDev, "laser_noise_stddev");
CONFIG_FLOAT(cAngularDriftRate, "angular_drift_rate");
CONFIG_FLOAT(cAngularErrorRate, "angular_error_rate");
CONFIG_FLOAT(cMaxLaserRange, "max_laser_range");
CONFIG_FLOAT(cLaserAngleIncrement, "laser_angle_increment");
CONFIG_FLOAT(cLaserFOV, "laser_fov");
config_reader::ConfigReader reader({kUtAutomataDir + "/config/simulator.lua"});

string MapNameToFile(const string& map) {
  return kAmrlMapsDir + "/" + map + "/" + map + ".vectormap.txt";
}

Simulator::Simulator() :
    random_(GetMonotonicTime() * 1e6),
    odom_loc_(random_.UniformRandom(-10, 10),
              random_.UniformRandom(-10, 10)),
    odom_angle_(random_.UniformRandom(-M_PI, M_PI)) {
  t_last_cmd_ = GetMonotonicTime();
  truePoseMsg.header.frame_id = "map";
}

Simulator::~Simulator() { }

void Simulator::ResetState() {
  odom_loc_ = Vector2f(random_.UniformRandom(-10, 10),
                       random_.UniformRandom(-10, 10));
  odom_angle_ = random_.UniformRandom(-M_PI, M_PI);
  true_robot_loc_ = Vector2f(cStartX, cStartY);
  true_robot_angle_ = cStartAngle;
  map_name_ = cMapName;
  map_.Load(MapNameToFile(cMapName));
}

void Simulator::Init(const rclcpp::Node::SharedPtr& node) {
  node_ = node;
  if (kAmrlMapsDir.empty()) {
    fprintf(stderr,
            "ERROR: AMRL maps directory not found. "
            "Make sure the amrl_maps package is built in the same colcon "
            "workspace and the install space is sourced.\n");
    exit(1);
  }
  scan_msg_.header.frame_id = "base_laser";
  scan_msg_.angle_min = -0.5 * cLaserFOV;
  scan_msg_.angle_max = 0.5 * cLaserFOV;
  scan_msg_.range_min = 0.02;
  scan_msg_.range_max = cMaxLaserRange;
  scan_msg_.angle_increment = cLaserAngleIncrement;
  scan_msg_.intensities.clear();
  scan_msg_.time_increment = 0.0;
  scan_msg_.scan_time = 0.05;

  odom_msg_.header.frame_id = "odom";
  odom_msg_.child_frame_id = "base_footprint";

  ResetState();
  InitSimulatorVizMarkers();
  DrawMap();

  drive_subscriber_ = node_->create_subscription<AckermannCurvatureDriveMsg>(
      "/ackermann_curvature_drive",
      1,
      std::bind(&Simulator::DriveCallback, this, std::placeholders::_1));
  init_subscriber_ = node_->create_subscription<amrl_msgs::msg::Localization2DMsg>(
      "/set_pose", 1, std::bind(&Simulator::InitalLocationCallback, this, std::placeholders::_1));
  odometry_publisher_ = node_->create_publisher<nav_msgs::msg::Odometry>("/odom", 1);
  laser_publisher_ = node_->create_publisher<sensor_msgs::msg::LaserScan>("/scan", 1);
  map_publisher_ = node_->create_publisher<visualization_msgs::msg::Marker>(
      "/simulator_visualization", 6);
  robot_marker_publisher_ = node_->create_publisher<visualization_msgs::msg::Marker>(
      "/simulator_visualization", 6);
  true_pose_publisher_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/simulator_true_pose", 1);
  if (FLAGS_localize) {
    localization_publisher_ = node_->create_publisher<amrl_msgs::msg::Localization2DMsg>(
        "/localization", 1);
    localization_msg_.header.frame_id = "map";
  }
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*node_);
}

void Simulator::InitalLocationCallback(
    const amrl_msgs::msg::Localization2DMsg::SharedPtr msg) {
  true_robot_loc_ = Vector2f(msg->pose.x, msg->pose.y);
  true_robot_angle_ = msg->pose.theta;
  if (map_name_ != msg->map) {
    map_.Load(MapNameToFile(msg->map));
    map_name_ = msg->map;
    DrawMap();
  }
  printf("Set robot pose: %.2f,%.2f, %.1f\u00b0 @ %s\n",
         true_robot_loc_.x(),
         true_robot_loc_.y(),
         RadToDeg(true_robot_angle_),
         msg->map.c_str());
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
void Simulator::InitVizMarker(
    visualization_msgs::msg::Marker& vizMarker,
    string ns,
    int id,
    string type,
    geometry_msgs::msg::PoseStamped p,
    geometry_msgs::msg::Point32 scale,
    double duration,
    vector<float> color) {

  vizMarker.header.frame_id = p.header.frame_id;
  vizMarker.header.stamp = node_->now();

  vizMarker.ns = ns;
  vizMarker.id = id;

  if (type == "arrow") {
    vizMarker.type = visualization_msgs::msg::Marker::ARROW;
  } else if (type == "cube") {
    vizMarker.type = visualization_msgs::msg::Marker::CUBE;
  } else if (type == "sphere") {
    vizMarker.type = visualization_msgs::msg::Marker::SPHERE;
  } else if (type == "cylinder") {
    vizMarker.type = visualization_msgs::msg::Marker::CYLINDER;
  } else if (type == "linelist") {
    vizMarker.type = visualization_msgs::msg::Marker::LINE_LIST;
  } else if (type == "linestrip") {
    vizMarker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  } else if (type == "points") {
    vizMarker.type = visualization_msgs::msg::Marker::POINTS;
  } else {
    vizMarker.type = visualization_msgs::msg::Marker::ARROW;
  }

  vizMarker.pose = p.pose;
  vizMarker.points.clear();
  vizMarker.scale.x = scale.x;
  vizMarker.scale.y = scale.y;
  vizMarker.scale.z = scale.z;

  vizMarker.lifetime.sec = static_cast<int32_t>(duration);
  vizMarker.lifetime.nanosec =
      static_cast<uint32_t>((duration - vizMarker.lifetime.sec) * 1e9);

  vizMarker.color.r = color.at(0);
  vizMarker.color.g = color.at(1);
  vizMarker.color.b = color.at(2);
  vizMarker.color.a = color.at(3);

  vizMarker.action = visualization_msgs::msg::Marker::ADD;
}

void Simulator::InitSimulatorVizMarkers() {
  geometry_msgs::msg::PoseStamped p;
  geometry_msgs::msg::Point32 scale;
  vector<float> color;
  color.resize(4);

  p.header.frame_id = "map";

  p.pose.orientation.w = 1.0;
  scale.x = 0.05;
  scale.y = 0.0;
  scale.z = 0.0;
  color[0] = 66.0 / 255.0;
  color[1] = 134.0 / 255.0;
  color[2] = 244.0 / 255.0;
  color[3] = 1.0;
  InitVizMarker(
      line_list_marker_, "map_lines", 0, "linelist", p, scale, 0.0, color);
  line_list_marker_.header.frame_id = "map";

  p.pose.position.z = 0.5 * cCarHeight;
  scale.x = cCarLength;
  scale.y = cCarWidth;
  scale.z = cCarHeight;
  color[0] = 94.0 / 255.0;
  color[1] = 156.0 / 255.0;
  color[2] = 255.0 / 255.0;
  color[3] = 0.8;
  InitVizMarker(
      robot_pos_marker_, "robot_position", 1, "cube", p, scale, 0.0, color);
  robot_pos_marker_.header.frame_id = "map";
}

void Simulator::DrawMap() {
  ros_helpers::ClearMarker(&line_list_marker_);
  for (const Line2f& l : map_.lines) {
    ros_helpers::DrawEigen2DLine(l.p0, l.p1, &line_list_marker_);
  }
}

void Simulator::PublishOdometry() {
  odom_msg_.header.stamp = node_->now();
  odom_msg_.pose.pose.position.x = odom_loc_.x();
  odom_msg_.pose.pose.position.y = odom_loc_.y();
  odom_msg_.pose.pose.position.z = 0.0;
  odom_msg_.pose.pose.orientation.x = 0;
  odom_msg_.pose.pose.orientation.y = 0;
  odom_msg_.pose.pose.orientation.z = sin(0.5 * odom_angle_);
  odom_msg_.pose.pose.orientation.w = cos(0.5 * odom_angle_);
  odom_msg_.twist.twist.angular.x = 0.0;
  odom_msg_.twist.twist.angular.y = 0.0;
  odom_msg_.twist.twist.angular.z = robot_ang_vel_;
  odom_msg_.twist.twist.linear.x = robot_vel_;
  odom_msg_.twist.twist.linear.y = 0;
  odom_msg_.twist.twist.linear.z = 0.0;

  odometry_publisher_->publish(odom_msg_);

  robot_pos_marker_.pose.position.x =
      true_robot_loc_.x() - cos(true_robot_angle_) * cRearAxleOffset;
  robot_pos_marker_.pose.position.y =
      true_robot_loc_.y() - sin(true_robot_angle_) * cRearAxleOffset;
  robot_pos_marker_.pose.position.z = 0.5 * cCarHeight;
  robot_pos_marker_.pose.orientation.x = 0;
  robot_pos_marker_.pose.orientation.y = 0;
  robot_pos_marker_.pose.orientation.z = sin(0.5 * true_robot_angle_);
  robot_pos_marker_.pose.orientation.w = cos(0.5 * true_robot_angle_);
}

void Simulator::PublishLaser() {
  scan_msg_.header.stamp = node_->now();
  const Vector2f laserRobotLoc(cLaserLocX, cLaserLocY);
  const Vector2f laserLoc = true_robot_loc_ + Rotation2Df(true_robot_angle_) * laserRobotLoc;

  const int num_rays = static_cast<int>(
      1.0 + (scan_msg_.angle_max - scan_msg_.angle_min) /
      scan_msg_.angle_increment);
  map_.GetPredictedScan(laserLoc,
                        scan_msg_.range_min,
                        scan_msg_.range_max,
                        scan_msg_.angle_min + true_robot_angle_,
                        scan_msg_.angle_max + true_robot_angle_,
                        num_rays,
                        &scan_msg_.ranges);
  for (float& r : scan_msg_.ranges) {
    if (r > scan_msg_.range_max - 0.1) {
      r = scan_msg_.range_max;
      continue;
    }
    r = max<float>(0.0, r + random_.Gaussian(0, cLaserStdDev));
  }
  laser_publisher_->publish(scan_msg_);
}

void Simulator::PublishTransform() {
  const auto now = node_->now();

  geometry_msgs::msg::TransformStamped map_to_odom;
  map_to_odom.header.stamp = now;
  map_to_odom.header.frame_id = "map";
  map_to_odom.child_frame_id = "odom";
  map_to_odom.transform.rotation.w = 1.0;
  tf_broadcaster_->sendTransform(map_to_odom);

  geometry_msgs::msg::TransformStamped odom_to_base_footprint;
  odom_to_base_footprint.header.stamp = now;
  odom_to_base_footprint.header.frame_id = "odom";
  odom_to_base_footprint.child_frame_id = "base_footprint";
  odom_to_base_footprint.transform.translation.x = true_robot_loc_.x();
  odom_to_base_footprint.transform.translation.y = true_robot_loc_.y();
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, true_robot_angle_);
  odom_to_base_footprint.transform.rotation.x = q.x();
  odom_to_base_footprint.transform.rotation.y = q.y();
  odom_to_base_footprint.transform.rotation.z = q.z();
  odom_to_base_footprint.transform.rotation.w = q.w();
  tf_broadcaster_->sendTransform(odom_to_base_footprint);

  geometry_msgs::msg::TransformStamped base_footprint_to_base_link;
  base_footprint_to_base_link.header.stamp = now;
  base_footprint_to_base_link.header.frame_id = "base_footprint";
  base_footprint_to_base_link.child_frame_id = "base_link";
  base_footprint_to_base_link.transform.rotation.w = 1.0;
  tf_broadcaster_->sendTransform(base_footprint_to_base_link);

  geometry_msgs::msg::TransformStamped base_link_to_laser;
  base_link_to_laser.header.stamp = now;
  base_link_to_laser.header.frame_id = "base_link";
  base_link_to_laser.child_frame_id = "base_laser";
  base_link_to_laser.transform.translation.x = cLaserLocX;
  base_link_to_laser.transform.translation.y = cLaserLocY;
  base_link_to_laser.transform.translation.z = cLaserLocZ;
  base_link_to_laser.transform.rotation.w = 1.0;
  tf_broadcaster_->sendTransform(base_link_to_laser);
}

void Simulator::PublishVisualizationMarkers() {
  map_publisher_->publish(line_list_marker_);
  robot_marker_publisher_->publish(robot_pos_marker_);
}

float AbsBound(float x, float bound) {
  if (x > 0.0 && x > bound) {
    return bound;
  } else if (x < 0.0 && x < -bound) {
    return -bound;
  }
  return x;
}

void Simulator::DriveCallback(const AckermannCurvatureDriveMsg::SharedPtr msg) {
 if (!isfinite(msg->velocity) || !isfinite(msg->curvature)) {
    printf("Ignoring non-finite drive values: %f %f\n",
           msg->velocity,
           msg->curvature);
    return;
  }
  last_cmd_ = *msg;
  t_last_cmd_ = GetMonotonicTime();
}

void Simulator::Update() {
  static const double kMaxCommandAge = 0.1;
  if (!step_mode_ && GetMonotonicTime() > t_last_cmd_ + kMaxCommandAge) {
    last_cmd_.velocity = 0;
  }
  const float dt = cSubSampleRate / cPublishRate;

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

  const float dv_max = dt * cMaxAccel;
  const float bounded_dv = AbsBound(desired_vel - robot_vel_, dv_max);
  robot_vel_ = robot_vel_ + bounded_dv;
  const float dist = robot_vel_ * dt;

  // Robot-frame uncorrupted motion.
  float dtheta;
  Vector2f dLoc;
  if (linear_motion) {
    dLoc = Vector2f(dist, 0);
    dtheta = 0;
  } else {
    const float r = 1.0 / desired_curvature;
    dtheta = dist * desired_curvature;
    dLoc = r * Vector2f(sin(dtheta), 1.0 - cos(dtheta));
  }

  odom_loc_ += Rotation2Df(odom_angle_) * dLoc;
  odom_angle_ = AngleMod(odom_angle_ + dtheta);

  true_robot_loc_ += Rotation2Df(true_robot_angle_) * dLoc;
  true_robot_angle_ = AngleMod(true_robot_angle_ + dtheta +
      dist * cAngularDriftRate +
      random_.Gaussian(0.0, fabs(dist) * cAngularErrorRate));

  truePoseMsg.header.stamp = node_->now();
  truePoseMsg.pose.position.x = true_robot_loc_.x();
  truePoseMsg.pose.position.y = true_robot_loc_.y();
  truePoseMsg.pose.position.z = 0;
  truePoseMsg.pose.orientation.w = cos(0.5 * true_robot_angle_);
  truePoseMsg.pose.orientation.z = sin(0.5 * true_robot_angle_);
  truePoseMsg.pose.orientation.x = 0;
  truePoseMsg.pose.orientation.y = 0;
  true_pose_publisher_->publish(truePoseMsg);
}

void Simulator::RunIteration() {
  // Simulate time-step.
  Update();

  if (last_publish_time_ < GetMonotonicTime() - 1.0 / cPublishRate) {
      //publish odometry and status
    PublishOdometry();
    //publish laser rangefinder messages
    PublishLaser();
    // publish visualization marker messages
    PublishVisualizationMarkers();
    //publish tf
    PublishTransform();
    last_publish_time_ = GetMonotonicTime();
  }

  if (FLAGS_localize) {
    localization_msg_.pose.x = true_robot_loc_.x();
    localization_msg_.pose.y = true_robot_loc_.y();
    localization_msg_.pose.theta = true_robot_angle_;
    localization_msg_.map = map_name_;
    localization_msg_.header.stamp = node_->now();
    localization_publisher_->publish(localization_msg_);
  }
}

void Simulator::Run() {
  // main loop
  const double simulator_fps = cPublishRate / cSubSampleRate;
  RateLoop rate(simulator_fps);
  while (rclcpp::ok()){
    rclcpp::spin_some(node_);
    RunIteration();
    rate.Sleep();
  }
}

void Simulator::Step(const amrl_msgs::msg::AckermannCurvatureDriveMsg& cmd,
                     nav_msgs::msg::Odometry* odom_msg,
                     sensor_msgs::msg::LaserScan* scan_msg,
                     amrl_msgs::msg::Localization2DMsg* localization_msg) {
  step_mode_ = true;
  last_cmd_ = cmd;
  t_last_cmd_ = GetMonotonicTime();
  const int num_iterations = ceil(1.0 / cSubSampleRate);
  for (int i = 0; i < num_iterations; ++i) {
    RunIteration();
  }
  *odom_msg = odom_msg_;
  *scan_msg = scan_msg_;
  *localization_msg = localization_msg_;
}

void Simulator::SetStepMode(bool step_mode) {
  step_mode_ = step_mode;
}