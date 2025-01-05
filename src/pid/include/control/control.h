#include "geometry_msgs/Twist.h"
#include "pid.hpp"
#include "ros/ros.h"
#include "std_msgs/Int64.h"
#include <iostream>

using namespace std;

class Control {
private:
    ros::NodeHandle nh;
    ros::Publisher control_pub;
    geometry_msgs::Twist msg;

public:
    Camera Transfer(double& x_axis);
    Control::Control(ros::NodeHandle& nh);
}
