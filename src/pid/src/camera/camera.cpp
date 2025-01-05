#include "camera/camera.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "camera_publisher");
    ros::NodeHandle nh;
    ros::Rate loop_rate(50);
    Camera Webcam(nh);
    while (ros::ok()) {
        Webcam.cam >> Webcam.src;
        Webcam.cam >> Webcam.result;
        Webcam.ToHSV(Webcam.src);
        Webcam.Make_contours(Webcam.src);
        Webcam.Compare();
        imshow("result", Webcam.result);
        waitKey(20);
    }
}
