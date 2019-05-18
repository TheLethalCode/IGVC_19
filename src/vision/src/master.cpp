#include <iostream>
#include <math.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/mat.hpp>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Image.h>

/*
   Custom header files
 */
#include "lane_segmentation.hpp"
#include "ransac.hpp"
//#include "mlesac.hpp"
#include "shadowRemoval.hpp"
// #include "beginner_tutorials/model.h"
#include "basic.hpp"
#include "find_pothole.hpp"
// #include "obsplot.hpp"
#include "cv_bridge/cv_bridge.h"
#include "lidar_new.hpp"
#include "waypts1_header.hpp"
// #include "combinedobs.hpp"
//#include "object.hpp"
// #include <grid_object.hpp>
//#include "white_obstacle.hpp"
#include "matrixTransformation.hpp"
#include "obstacles_prev.hpp"

#define ppm 50

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
        for(float r=msg.range_min;r<=msg.range_max;r+=1.0/ppm)
        {
            cout<<"loop"<<endl;
            int i=img.rows-r*cos(theta)*ppm;
            int j=img.cols/2-r*sin(theta)*ppm;
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
    Mat inflated; 
    model lanes;		//For Ransac implementation(it is a structure)
    lanes.a1=0;lanes.b1=0;lanes.c1=0;lanes.a2=0;lanes.b2=0;lanes.c2=0;

    init(argc,argv,"master");
    NodeHandle n;
    image_transport::ImageTransport it(n);

    Publisher waypoint_publisher = n.advertise<geometry_msgs::Point>("waypoint",1000);

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

        /*
           int ic=roi.rows/2,jc=roi.cols/2;
           int r;
           for(int i=0;i<roi.rows;i++)
           {
           for(int j=0;j<roi.cols;j++)
           {

           r=sqrt((i-ic)*(i-ic)+(j-jc)*(j-jc));
           roi.at<Vec3b>(i,j)=roi.at<Vec3b>(i,j)/(1-(float)(0.9*r/mn(roi.rows,roi.cols)));
           }
           }
         */


        //removing shadows
        /*
        roi=removeShadow(roi);
        if (true) {
            namedWindow("shadowRemoval", WINDOW_NORMAL);
            imshow("shadowRemoval",roi); 
            waitKey(10);
        }
        */



        // for(int u = 0; u < roi.rows; u++)
        // {
        //     for(int v = 0; v < roi.cols; v++)
        //     {
        //         roi.at<Vec3b>(u, v)[0] = min(roi.at<Vec3b>(u, v)[0]+20, 255);
        //         roi.at<Vec3b>(u, v)[1] = min(roi.at<Vec3b>(u, v)[1]+20, 255);
        //         roi.at<Vec3b>(u, v)[2] = min(roi.at<Vec3b>(u, v)[2]+40, 255);

        //     }
        // }

        // Mat roi=object_remove_2(&roi);
        // namedWindow("obstacle",0);
        // imshow("obstacle",roi); 
        //	spinOnce();
        //	continue;

        //extraction of different channels
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

        medianBlur(intersectionImages, intersectionImages, 31);


        if (true) {
            namedWindow("intersectionImages", WINDOW_NORMAL);
            imshow("intersectionImages", intersectionImages);
            waitKey(10);
        }

        //       namedWindow("unionImages",0);
        // imshow("unionImages",unionImages); 
        //	spinOnce();
        //	continue;

        // for(int i=0;i<roi.rows;i++)
        // {
        //     for(int j=0;j<roi.cols;j++)
        //     { 
        //         if(obstacle.at<uchar>(i,j)==255)
        //         {
        //             unionImages.at<uchar>(i,j)=0;
        //         }
        //     }
        // }  

        Mat topView = top_view(intersectionImages);

        if (true) {
            namedWindow("top_view",WINDOW_NORMAL);	
            imshow("top_view",topView); 
            waitKey(10);
        }


        // Mat pot_hole=find_pothole(unionImages);
        // namedWindow("pot_hole",0);
        // imshow("pot_hole",pot_hole); 
        //	spinOnce();
        //	continue;

        // img_to_ls(pot_hole);
        //	spinOnce();
        //	continue;

        // for(int i=0;i<roi.rows;i++)
        // {
        //     for(int j=0;j<roi.cols;j++)
        //     {
        //			if(pot_hole.at<uchar>(i,j)==255)
        //         {
        //             unionImages.at<uchar>(i,j)=0;
        //         }
        //     }
        // } 
        // namedWindow("pot_hole_removed",0);
        // imshow("pot_hole_removed",unionImages); 
        //	spinOnce();
        //	continue; 

        // curve fitting
        lanes=getRansacModel(topView,lanes);
        Mat fitLanes = drawLanes(topView, lanes);

        if (true) {
            namedWindow("lanes ftting", WINDOW_NORMAL);
            imshow("lanes fitting", fitLanes);
            waitKey(10);
        }

        spinOnce();
        continue;
        // img_to_ls(lanes_by_ransac);
        //	spinOnce();
        //	continue;

        /*
        namedWindow("lidar_plot",0);
        imshow("lidar_plot",lidar_plot); 
        inflated=obstaclePlot;
        namedWindow("lidar_inflated_obs",0);
        imshow("lidar_inflated_obs",obstaclePlot); 
        */

        output waypoint=find_waypoint(lanes,inflated);

        geometry_msgs::Point msg1;
        msg1.x=waypoint.x;
        msg1.y=waypoint.y;
        msg1.z=waypoint.angle;

        // beginner_tutorials::mat msg2;
        // waypoint_publisher.publish(msg1);
        //subscribe->from costmap

        waitKey(30);
        spinOnce();
    }
    return 0;
}
