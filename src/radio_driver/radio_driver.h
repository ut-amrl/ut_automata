#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

namespace radio_driver {

class RadioDriver {
   public:
    RadioDriver(ros::NodeHandle nh);

   private:
    void joystickCallback(const sensor_msgs::Joy& msg);
    void timerCallback(const ros::SteadyTimerEvent& event);

    ros::Subscriber joystick_sub_;
    ros::SteadyTimer timer_;
};

}  // namespace radio_driver