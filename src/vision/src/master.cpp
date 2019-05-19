#include <iostream>
#include <math.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/mat.hpp>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Image.h>
#include "geometry_msgs/PoseStamped.h"
#include <tf/transform_datatypes.h>

/*
   Custom header files
 */

#include <ransac.hpp>
#include <lane_segmentation.hpp>
#include <waypoint_generator.hpp>

#include <lidar_new.hpp>
#include <obstacles_prev.hpp>

#include <matrixTransformation.hpp>

#define PPM 50

using namespace std;
using namespace cv;
using namespace ros;

Publisher ls;
Mat frame_orig;

bool is_image_retrieved = false;
bool use_video = false;

int mn(int a,int b)
{
    if(a<b)
        return a;
    else
        return b;
}

void img_to_ls(Mat img)
{
    sensor_msgs::LaserScan msg;
    msg.angle_min=-CV_PI/2;
    msg.angle_max=CV_PI/2;
    // msg.range_min=0.5;
    // msg.range_max=4;
    int k=0,l=0;
    msg.angle_increment=CV_PI/180;
    for(float theta=msg.angle_min;theta<=msg.angle_max;theta+=msg.angle_increment)
    {
        l=0;
        for(float r=msg.range_min;r<=msg.range_max;r+=1.0/PPM)
        {
            cout<<"loop"<<endl;
            int i=img.rows-r*cos(theta)*PPM;
            int j=img.cols/2-r*sin(theta)*PPM;
            if(img.at<uchar>(i,j)==255)
            {
                msg.ranges[k]=r;
                l++;
                break;
            }
        }
        if(l==0)
            msg.ranges[k]=msg.range_max;
        k++;


    }
    ls.publish(msg);
}

void imageCb(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;

    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);    
        is_image_retrieved=true;
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
    frame_orig = (cv_ptr->image);

}

int main(int argc, char **argv)
{ 

    Mat obstacle;
    Mat costmap; 
    Parabola lanes;		//For Ransac implementation(it is a structure)
    lanes.a1=0;lanes.b1=0;lanes.c1=0;lanes.a2=0;lanes.b2=0;lanes.c2=0;

    init(argc,argv,"master");
    NodeHandle n;
    image_transport::ImageTransport it(n);

    Publisher waypoint_publisher = n.advertise<geometry_msgs::PoseStamped>("waypoint",1000);

    Subscriber lidar_subsriber;
    if (use_video == false) {
        lidar_subsriber = n.subscribe("/scan", 10, &laserscan);
    }

    image_transport::Subscriber sub = it.subscribe("/camera/image_color", 1000, imageCb);

    ls = n.advertise<sensor_msgs::LaserScan>("ls", 1000);       //declared globally


    while(ros::ok())
    {
        //if image has not been retrieved, skip
        if(!is_image_retrieved)
        {
            spinOnce();
            continue;
        }

        //if lidar has not been retrieved, skip
        if(!is_laserscan_retrieved && !use_video)
        {
            spinOnce();
            continue;
        }

        Mat roi;
        Mat img1,img2,img3;     //temporary images for using filters
        Mat b_processed;
        Mat bg_processed;
        Mat br_processed;

        if (true) {
            namedWindow("original",WINDOW_NORMAL);
            imshow("original", frame_orig);
        }

        /*
           ROI
         */

        //extraction of region of interest
        Rect roi_rect = Rect(0, 0, frame_orig.cols, frame_orig.rows); //params in order: x, y, height, width (of ROI)
        roi = frame_orig(roi_rect);

        if (true) {
            namedWindow("roi",WINDOW_NORMAL);
            imshow("roi",roi); 
        }

        //processing done for various channels

        roi = remove_obstacles(roi);

        if (true) {
            namedWindow("obstacles removed", WINDOW_NORMAL);
            imshow("obstacles removed", roi);
            waitKey(10);
        }

        Mat twob_r = twob_rChannelProcessing(roi);

        if (false) {
            namedWindow("2b-r", WINDOW_NORMAL);
            imshow("2b-r", twob_r);
            waitKey(10);
        }

        Mat twob_g = twob_gChannelProcessing(roi);

        if (false) {
            namedWindow("2g", WINDOW_NORMAL);
            imshow("2g", twob_g);
            waitKey(10);
        }


        //processing for blue channel
        Mat b = blueChannelProcessing(roi);

        if (false) {
            namedWindow("b", WINDOW_NORMAL);
            imshow("b", b);
            waitKey(10);
        }

        //union of all lane filters
        Mat unionImages;
        bitwise_or(twob_r, twob_g, unionImages);
        bitwise_or(unionImages, b, unionImages);


        if (false) {
            namedWindow("unionImages", WINDOW_NORMAL);
            imshow("unionImages", unionImages);
            waitKey(10);
        }

        //intersection of all lane filters
        Mat intersectionImages;
        bitwise_and(twob_r, twob_g, intersectionImages);
        bitwise_and(intersectionImages, b, intersectionImages);

//        medianBlur(intersectionImages, intersectionImages, 31);

        if (true) {
            namedWindow("intersectionImages", WINDOW_NORMAL);
            imshow("intersectionImages", intersectionImages);
            waitKey(10);
        }

        Mat topView = top_view(intersectionImages);

        if (true) {
            namedWindow("top_view",WINDOW_NORMAL);	
            imshow("top_view",topView); 
            waitKey(10);
        }

        // Add top view preprocessed image to costmap
        // img_to_ls(topView);

        // curve fitting
        lanes = getRansacModel(topView,lanes);
        Mat fitLanes = drawLanes(topView, lanes);

        if (true) {
            namedWindow("lanes fitting", WINDOW_NORMAL);
            imshow("lanes fitting", fitLanes);
            waitKey(10);
        }

        //plot obstacles on fitLanes and then pass fitLanes to find_waypoint 
        /*
        namedWindow("lidar_plot",0);
        imshow("lidar_plot",lidar_plot); 
        costmap=obstaclePlot;
        namedWindow("lidar_costmap_obs",0);
        imshow("lidar_costmap_obs",obstaclePlot); 
        */

        costmap = fitLanes.clone();


        //return waypoint assuming origin at bottom left of image (in pixel coordinates)
        NavPoint waypoint_image = find_waypoint(lanes,costmap); //in radians
        costmap = plotWaypoint(costmap, waypoint_image);

        if (true) {
            namedWindow("waypoint", WINDOW_NORMAL);
            imshow("waypoint", costmap);
            waitKey(10);
        }

        //transforming waypoint to ros convention (x forward, y left, angle from x and positive clockwise) (in metres)
        geometry_msgs::PoseStamped waypoint_bot;

        waypoint_bot.pose.position.x = waypoint_image.y/PPM;
        waypoint_bot.pose.position.y = (costmap.cols/2 - waypoint_image.x)/PPM;
        waypoint_bot.pose.position.z = 0;
        float theta = (waypoint_image.angle - CV_PI/2);

        tf::Quaternion frame_qt = tf::createQuaternionFromYaw(theta);
        waypoint_bot.pose.orientation.x = frame_qt.x();
        waypoint_bot.pose.orientation.y = frame_qt.y();
        waypoint_bot.pose.orientation.z = frame_qt.z();
        waypoint_bot.pose.orientation.w = frame_qt.w();

        waypoint_publisher.publish(waypoint_bot);

        waitKey(30);
        spinOnce();
    }
    return 0;
}
