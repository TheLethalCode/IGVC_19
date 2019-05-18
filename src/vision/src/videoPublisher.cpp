#include "ros/ros.h"
#include "std_msgs/String.h"
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;
using namespace ros;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "videoPublisher");

    ros::NodeHandle n;
    image_transport::ImageTransport it(n);

    image_transport::Publisher pub = it.advertise("/camera/image_color", 100);

    ros::Rate loop_rate(10);

    String path = "/home/mechanical/igvc.mp4";
    VideoCapture cap(path);
    Mat frame;

    cap >> frame;

    while (ros::ok() && !frame.empty())
    {
        cap >> frame;

        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
        pub.publish(msg);

        ros::spinOnce();
        //loop_rate.sleep();
        waitKey(10);
    }


    return 0;
}
