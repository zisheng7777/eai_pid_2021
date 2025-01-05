#include "camera/camera.h"

Camera::Camera(ros::NodeHandle& nh)
    : pid(0.001, 0, 0, 0)
{
    cam.open("/dev/v4l/by-id/usb-046d_0821_418A61A0-video-index0");
    if (!cam.isOpened()) {
        ROS_ERROR("\ncamera not opened\n");
        ROS_INFO("\nClass Camera constructed\n");
    }
    cam.set(CAP_PROP_FRAME_WIDTH, 640);
    cam.set(CAP_PROP_FRAME_HEIGHT, 480);
    cam >> src;
    camera_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
}

void Camera::ToHSV(Mat& hsv)
{
    cvtColor(hsv, hsv, COLOR_RGB2HSV);
    Scalar lower = Scalar(0, 150, 50);
    Scalar upper = Scalar(180, 255, 255);
    inRange(hsv, lower, upper, hsv);
}
void Camera::Make_contours(Mat& src)
{
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;

    findContours(src, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);

    std::vector<std::vector<cv::Point>>::iterator itc = contours.begin();
    std::vector<cv::Vec4i>::iterator itc_hierarchy = hierarchy.begin();

    int min_size = 60;
    int max_size = 500;

    //畫面中心
    Point base = Point(320, 240);
    circle(result, base, 5, Scalar(255, 0, 0), -1);

    // 針對所有輪廓
    itc = contours.begin();
    itc_hierarchy = hierarchy.begin();
    while (itc != contours.end()) {
        //驗證輪廓大小
        if (itc->size() < min_size) {
            itc = contours.erase(itc);
            itc_hierarchy = hierarchy.erase(itc_hierarchy);
        } else {
            ++itc;
            ++itc_hierarchy;
        }
    }

    vector<Rect> boundRect(contours.size());
    int j = 0;

    for (int i = 0; i < contours.size(); i++) {
        Rect rect = boundingRect(contours[i]);
        int compare = 0;
        if (compare < rect.area()) {
            compare = rect.area();
        } else {
            j = i;
        }
        boundRect[j] = boundingRect(contours[j]);
        rectangle(result, boundRect[j], Scalar(0, 255, 0));

        // 畫物件中心
        Point pt = Point((boundRect[j].x + (boundRect[j].width / 2)), (boundRect[j].y + (boundRect[j].height / 2)));

        circle(result, pt, 2, Scalar(0, 0, 255), 2);
        // 標註重心座標值
        string text_x = to_string(pt.x);
        string text_y = to_string(pt.y);
        string text = "(" + text_x + ", " + text_y + ")";
        putText(result, text, Point(pt.x + 10, pt.y + 10), FONT_HERSHEY_PLAIN, 1.5, Scalar::all(255), 1, 8, 0);
        x_axis = boundRect[j].x + (boundRect[j].width / 2);
    }
}

void Camera::Compare()
{
    pid.init();
    x_axis = x_axis - 320;
    pid_value = pid.pidCtrl(-x_axis, 0);
    if (pid_value != 0.32) {
        msg.linear.x = 0;
        msg.angular.z = pid_value;
        camera_pub.publish(msg);
    }
}
