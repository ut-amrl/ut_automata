#include <ros/ros.h>
#include <utility>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Bool.h>
#include <tf/transform_broadcaster.h>//there is a more specific lib
#include <libconfig.h>
#include "popt_pp.h"
#include "f1tenth_course/AckermannCurvatureDriveMsg.h"

using namespace std;
using f1tenth_course::AckermannCurvatureDriveMsg;

const bool kDebug = false;

// If the input method is "waypoint" listen to the published waypoints by
// navigation node and if the input method is set to "velocity" listen to
// directly to the desired velocity values (rather than computing velocity
// values based on the waypoint).
bool kListenToWaypoint = true;

// kDriveEnabled works as the emergency stop. If set to false velocity 
// commands will not be sent to the motor driver by this node.
bool kDriveEnabled = true;

// If kUseDeadManButton is set to true, in order for velocity commands to be
// sent to motor driver by this node two conditions should be met:
// 1- kDriveEnabled should be "true" (emergency stop released)
// 2- Deadman button should be engaged, i.e. have been held for more than 
//    kDeadManButtonHoldTime seconds.
const bool kUseDeadManButton = true;
const float kDeadManButtonHoldTime = 1.0; // sec
bool kDeadManButtonEngaged = false;


const float kWheelBase = 0.20; // meter
const float kCmdRate = 20; // Hz
const double kDebouncingDelay = 0.4; // seconds
const float kMinimumRefreshRate = 1.0; // (Hz) if the obtained command rate is 
                                       // less than this threshold. The command 
                                       // will be reset to zero

const double kTransAccel = 0.4; // forward acceleration m/s^2 0.8
const double kTransDecel = 0.8; // forward deceleration m/s^2 def: 0.4 for JPP. 
                                // 0.8 for kinect
const double kRotAccel = 10.0;//0.05; // rotational acceleration rad/s^2


// Parameters for waypoint following mode
const float kMaxLinVel = 0.3;//0.6; // maximum forward velocity
const double kMaxRotVel = 0.7;//1.3; // maximum rotational velocity
const double kRotationBias = 0.0;
const double kRotationalP = 1.0; // the proportional term in the rotational PD 
                           //controller
const float kDistThresh = 0.045;//0.045;//near enough distance


typedef nav_msgs::Path Path;
typedef geometry_msgs::Point GPoint;

// Global variables
Path path;
int path_index = -1;
geometry_msgs::Pose path_init_pose;
geometry_msgs::Pose current_pose;
tf::Pose tf_path_init_pose;
tf::Pose tf_current_pose;
geometry_msgs::Twist current_twist;
pair< double, double > latest_desired_vel;
double forward_vel = 0., rot_vel = 0.;

//defining path for rviz
vector< GPoint > rvizPath;

//publishers not in main
ros::Publisher vel_pub;
ros::Publisher vel_pub_ackermann;
ros::Publisher path_pub;



float CalculateSteeringAngle(float lin_vel, float rot_vel) {
  float steering_angle = 0.0;
  if (rot_vel == 0) {
    return steering_angle;
  }
  
  float turn_radius = lin_vel / rot_vel;
  steering_angle = atan(kWheelBase / turn_radius);
  return steering_angle;
}                                 
                                 
void DesiredVelCallBack(const geometry_msgs::Twist::ConstPtr& vel) {
  latest_desired_vel.first = vel->linear.x;
  latest_desired_vel.second = vel->angular.z;
}

void DesiredVelCallBackAckermann(const AckermannCurvatureDriveMsg& msg) {
if (!isfinite(msg.velocity) || !isfinite(msg.curvature)) {
    printf("Ignoring non-finite drive values: %f %f\n",
           msg.velocity,
           msg.curvature);
    return;
  }
  latest_desired_vel.first = msg.velocity;
  latest_desired_vel.second = msg.velocity * msg.curvature;
}

void newPathCallBack(const nav_msgs::Path::ConstPtr& p){
  //get the generated path

  path.header = p->header;
  path.poses = p->poses;
  path_index = path.poses.size() - 1;
  //ROS_INFO("path pose: %d", path.poses.size());
  path_init_pose = current_pose;
  tf_path_init_pose = tf_current_pose;


  //initulizing ros path marker
  visualization_msgs::Marker line_strip;
  line_strip.header.frame_id = "base_link";
  line_strip.header.stamp = ros::Time::now();
  line_strip.ns = "ros_path";
  line_strip.action = visualization_msgs::Marker::ADD;
  line_strip.pose.orientation.w = 1.0;
  line_strip.id = 0;
  line_strip.type = visualization_msgs::Marker::LINE_STRIP;
  line_strip.scale.x = 0.1;
  line_strip.color.b = 1.0;
  line_strip.color.a = 1.0;

  //fill in line_strip from path
  for(uint32_t i = 0; i < path.poses.size(); i++)
  {
    line_strip.points.push_back(path.poses[i].pose.position);
  }

  //publish
  path_pub.publish(line_strip);
}

//transforms a pose into the reference frame of the robot from odometry
geometry_msgs::Pose transform_pose(geometry_msgs::Pose pose)
{
  tf::Transform trans = tf_current_pose.inverseTimes(tf_path_init_pose);

  tf::Pose tf_pose;
  tf::poseMsgToTF(pose, tf_pose);

  tf_pose = trans*tf_pose;

  geometry_msgs::Pose new_pose;
  tf::poseTFToMsg(tf_pose, new_pose);//getting warnings


  return new_pose;
}

void odometry_call_back(const nav_msgs::Odometry& odom)
{
  current_twist = odom.twist.twist;
  current_pose = odom.pose.pose;
  tf::poseMsgToTF(odom.pose.pose, tf_current_pose);

  //initulizing ros path marker
  visualization_msgs::Marker line_strip;
  line_strip.header.frame_id = "base_link";
  line_strip.header.stamp = ros::Time::now();
  line_strip.ns = "ros_path";
  line_strip.action = visualization_msgs::Marker::ADD;
  line_strip.pose.orientation.w = 1.0;
  line_strip.id = 0;
  line_strip.type = visualization_msgs::Marker::LINE_STRIP;
  line_strip.scale.x = 0.1;
  line_strip.color.b = 1.0;
  line_strip.color.a = 1.0;

  //fill in line_strip from path
  for(uint32_t i = 0; i < path.poses.size(); i++)
  {
    line_strip.points.push_back(transform_pose(path.poses[i].pose).position);
  }

  //publish
  path_pub.publish(line_strip);
}

float distance_to_point(geometry_msgs::Point p){
  return sqrt(p.x*p.x + p.y*p.y);
}

float angle_to_point(geometry_msgs::Point p){
  return atan2(p.y, p.x);
}

pair< double, double > bestVelocity(float radius, int sign)
{
	pair< double, double > velocity (0,0);
	if (radius == -1)
	{
		velocity.first = kMaxLinVel;
	  velocity.second = 0;
	}
	else if (radius != 0)
	{
	   velocity.first = kMaxLinVel;
	   velocity.second = (kMaxLinVel/radius) * sign;
	}
	return velocity;
}

pair< double, double > goto_pose(geometry_msgs::Pose pose)
{
  //for now orientation is egnored
  pair< double, double > velocity(0,0);

  //ROS_INFO("pose.position.x: %f", pose.position.x);
  if (pose.position.x <= 0.01)
  {
    //ROS_INFO("stop!");
    velocity.first = 0.0;
  }
  else
  {
    //ROS_INFO("go as you please");
    velocity.first = kMaxLinVel;
  }

  // geometry_msgs::Pose gPose = transform_pose(pose);

  velocity.second = (kRotationalP * angle_to_point(pose.position));
  if (velocity.second > 0) {
    velocity.second = velocity.second + kRotationBias;
    velocity.second = min(velocity.second, kMaxRotVel);
  }else {
    velocity.second = velocity.second - kRotationBias;
    velocity.second = max(velocity.second, -kMaxRotVel);
  }
  if (kDebug) {
    ROS_INFO("Commanded angular velocity: %f\n", velocity.second);
  }
  return velocity;

}

pair< double, double > follow_path()
{
  //ROS_INFO("dist to point: %f\n", 
  // distance_to_point(path.poses[path_index].pose.position));
  //^causes segmentation fault if no path
  if (path_index < 0 || path.poses.size() <= 0)
  {
    return(pair< double, double > (0,0));
  }
  else if (distance_to_point(path.poses[path_index].pose.position) < 
kDistThresh)
  {
    path_index--;
    return follow_path();
  }
  else
  {
    // ROS_INFO("path.poses.size(): %d", path.poses.size());
    // ROS_INFO("index: %d", path_index);
    //transform_pose(path.poses[path_index]. pose));
    return goto_pose(path.poses[path_index].pose);
  }
  return(pair< double, double > (0,0));
}

void JoystickCallback(const sensor_msgs::JoyConstPtr& msg) {
  // read joystick input
  // Wired Xbox360 Controller
  int start_btn = msg->buttons[7];
  int deadman_btn = msg->buttons[5]; // RB

  
  static double last_start_press_time = 0;
  static bool deadman_btn_pressed = false;
  static double deadman_btn_press_time = 0;
  static double deadman_btn_hold_duration = 0;
  
  if (start_btn) {
    double curr_time = ros::Time::now().toSec();
    if ((curr_time - last_start_press_time) > kDebouncingDelay) {
      kDriveEnabled = !kDriveEnabled;
      last_start_press_time = ros::Time::now().toSec();
      ROS_INFO("DriveEnabled: %d\n", kDriveEnabled);
    }
  }
  
  // Deadman button should have been pressed for at least kDeadManButtonHoldTime
  // in order for it to be engaged
  if (deadman_btn) {
    if (!deadman_btn_pressed) {
      deadman_btn_press_time = ros::Time::now().toSec();
      deadman_btn_pressed = true;
    } 
    
    deadman_btn_hold_duration = ros::Time::now().toSec() 
                                  - deadman_btn_press_time;
    if (deadman_btn_hold_duration > kDeadManButtonHoldTime) {
      kDeadManButtonEngaged = true;
    }
  } else {
    deadman_btn_pressed = false;
    kDeadManButtonEngaged = false;
    deadman_btn_hold_duration = 0.0;
  }
}

void DriveEnableCallback(const std_msgs::Bool& msg) {
  kDriveEnabled = msg.data;
}

void SendCommand() {
  double desired_forward_vel, desired_rot_vel;
  pair< double, double > desired_vel;

  if (kListenToWaypoint) {
    desired_vel = follow_path();
  } else {
    desired_vel = latest_desired_vel;
  }

  if (kDebug) {
    ROS_INFO("wanted V: %f, W: %f", desired_vel.first, desired_vel.second);
  }

  // accelerate or decelerate accordingly
  static double prev_time = 0.0;
  double curr_time = ros::Time::now().toSec();
  double del_t = curr_time - prev_time;
  // Clamp del_t to a maximum value for cases when the button has not been
  // pressed for a while
  if (del_t > (1.0 / kMinimumRefreshRate)) {
    del_t = 0.05;
    forward_vel = 0.0;
    rot_vel = 0.0;
  }
  prev_time = curr_time;
  
  bool deadman_switch_ok = false;
  if ((kUseDeadManButton && kDeadManButtonEngaged) || !kUseDeadManButton) {
    deadman_switch_ok = true;
  } else {
    deadman_switch_ok = false;
  }

  if (kDriveEnabled && deadman_switch_ok) {
    desired_forward_vel = desired_vel.first;
    desired_rot_vel = desired_vel.second;
    if (desired_forward_vel < forward_vel) {
      forward_vel = max(desired_forward_vel, forward_vel - kTransDecel * del_t);
    } else {
      forward_vel = min(desired_forward_vel, forward_vel + kTransAccel * del_t);
    }
    if (desired_rot_vel < rot_vel) {
      rot_vel = max(desired_rot_vel, rot_vel - kRotAccel * del_t);
    } else {
      rot_vel = min(desired_rot_vel, rot_vel + kRotAccel * del_t);
    }

    if (kDebug) {
      ROS_INFO("real V: %f, W: %f", forward_vel, rot_vel);
    }
    geometry_msgs::Twist vel_msg;
    vel_msg.linear.x = forward_vel;
    vel_msg.angular.z = rot_vel;
    // vel_pub.publish(vel_msg);
    
    ackermann_msgs::AckermannDriveStamped vel_msg_ackermann;
    vel_msg_ackermann.header.stamp = ros::Time::now();
    vel_msg_ackermann.header.frame_id = "base_link";
    
    float steering_angle = CalculateSteeringAngle(forward_vel, rot_vel);
    
    vel_msg_ackermann.drive.speed = forward_vel;
    vel_msg_ackermann.drive.steering_angle = steering_angle;
    vel_pub_ackermann.publish(vel_msg_ackermann);
  } else {
    forward_vel = 0.0;
    rot_vel = 0.0;
    ROS_INFO("Drive is Disabled.");
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "motion_profiler");
  ros::NodeHandle nh;

  const char* input_method;
  static struct poptOption options[] = {
    { "input",'i',POPT_ARG_STRING,&input_method,0,"Input method"
      "- velocity, waypoint","STR" },
    POPT_AUTOHELP
    { NULL, 0, 0, NULL, 0, NULL, NULL }
  };

  POpt popt(NULL, argc, argv, options, 0);
  int c;
  while((c = popt.getNextOpt()) >= 0) {}

  if (strcmp(input_method, "velocity") == 0) {
    kListenToWaypoint = false;
  } else if (strcmp(input_method, "waypoint") == 0) {
    kListenToWaypoint = true;
  } else {
    printf("Input method should be either velocity or waypoint.\n");
    return 0;
  }

  latest_desired_vel.first = 0.0;
  latest_desired_vel.second = 0.0;

  path_pub = nh.advertise<visualization_msgs::Marker>("path_marker", 10);
  // vel_pub =  nh.advertise<geometry_msgs::Twist>(
  //                               "/jackal_velocity_controller/cmd_vel", 1);
  vel_pub_ackermann = nh.advertise<ackermann_msgs::AckermannDriveStamped>(
                                "/commands/ackermann", 1);


  ros::Subscriber sub_safe_drive = 
      nh.subscribe("/bluetooth_teleop/joy", 1, JoystickCallback);
  ros::Subscriber sub_path =
      nh.subscribe("/motion_profiler/planned_path", 1, newPathCallBack);
  ros::Subscriber sub_velocity_desired =
      nh.subscribe("/motion_profiler/desired_velocity", 1, DesiredVelCallBack);
  ros::Subscriber sub_velocity_desired_ackermann =
      nh.subscribe("/ackermann_curvature_drive", 
                    1, 
                    DesiredVelCallBackAckermann);
  ros::Subscriber sub_odometry = 
      nh.subscribe("/odom", 1, odometry_call_back);
  ros::Subscriber sub_drive_enable = 
      nh.subscribe("/motion_profiler/enable_drive", 1, DriveEnableCallback);
  
  ros::Rate rate(kCmdRate);
  while (ros::ok()) {
    ros::spinOnce();
    SendCommand();
    rate.sleep();
  }

  return 0;
}
