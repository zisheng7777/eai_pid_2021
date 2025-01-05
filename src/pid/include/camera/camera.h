#include "geometry_msgs/Twist.h"
#include "pid.hpp"
#include <iostream>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
using namespace std;
using namespace cv;

class Camera {
private:
    ros::NodeHandle nh_;
    ros::Publisher camera_pub;
    PID pid;
    double x_axis;
    double pid_value;
    geometry_msgs::Twist msg;

public:
    Mat src;
    Mat result;
    VideoCapture cam;
    Camera(ros::NodeHandle& nh);
    void ToHSV(Mat& hsv);
    void Make_contours(Mat& src);
    void Compare();
};
