#include <amrl_msgs/AckermannCurvatureDriveMsg.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

#include "radio_driver/radio_interface.h"

namespace radio_driver {

class RadioDriver {
   public:
    RadioDriver(ros::NodeHandle nh);

   private:
    enum class DriveMode {
        Disabled,
        Manual,
        Autonomous,
        ContinuousAutonomous,
    };

    ros::Publisher odom_pub_;
    ros::Publisher drive_pub_;
    ros::Publisher autonomy_enabler_pub_;

    ros::SteadyTimer timer_;
    void timerCallback(const ros::SteadyTimerEvent& event);

    ros::Subscriber ackermann_curvature_sub_;
    void ackermannCurvatureCallback(const amrl_msgs::AckermannCurvatureDriveMsg& msg);

    float ackermann_velocity_;
    float ackermann_curvature_;

    ros::Subscriber joystick_sub_;
    void joystickCallback(const sensor_msgs::Joy& msg);

    float joystick_velocity_;
    float joystick_curvature_;

    float speedToThrottle(float speed) const;
    float curvatureToSteering(float curvature) const;
    float clampVelocity(float velocity) const;
    float clampCurvature(float curvature) const;

    void updateOdometry();

    nav_msgs::Odometry odom_msg_;

    DriveMode drive_mode_;
    RadioInterface radio_interface_;

    float lastSpeed_;
    float lastCurvature_;
    float lastThrottle_;
    float lastSteering_;
};

}  // namespace radio_driver
