#include <iostream>
#include <math.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/mat.hpp>
#include "lane_segmentation.hpp"
//#include "object.hpp"
#include "ransac.hpp"
//#include "combinedobs.hpp"
//#include "mlesac.hpp"
#include "shadowRemoval.hpp"
// #include "beginner_tutorials/model.h"
#include "basic.hpp"
#include "find_pothole.hpp"
#include <geometry_msgs/Point.h>
#include <ros/ros.h>
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/Image.h"
#include "obsplot.hpp"
#include "cv_bridge/cv_bridge.h"
// #include "lidar_new.hpp"
#include "waypts1_header.hpp"
#include "combinedobs.hpp"
#include <image_transport/image_transport.h>
// #include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
// #include <grid_object.hpp>
#include "white_obstacle.hpp"

#define ppm 50

using namespace cv;
using namespace std;

Publisher ls;
Mat frame_orig;
bool MALI = false;

void imageCb(const sensor_msgs::ImageConstPtr& msg)
{

    cv_bridge::CvImagePtr cv_ptr;

  try
  {
       cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);    
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }

    // imshow("test", cv_ptr->image);
    // waitKey(10);
    frame_orig = (cv_ptr->image);

    // abc(cv_ptr->image);
    // imshow("frame_orig",frame_orig);
    // waitKey(0);
    // cout<<"abfhebjf"<<endl;
    MALI=true;

}

int main(int argc, char **argv)
{
   Mat obstacle;
    Mat inflated; 
    //int count=500;//frames to be skipped initially
    model lanes;//For Ransac implementation(it is a structure)
    lanes.a1=0;lanes.b1=0;lanes.c1=0;lanes.a2=0;lanes.b2=0;lanes.c2=0;


    init(argc,argv,"master");
    NodeHandle n;
    ls = n.advertise<sensor_msgs::LaserScan>("ls", 1000);
   
    // Publisher RANSAC_PUB= n.advertise<beginner_tutorials::model>("ransac_to_costmap",1000);
    Publisher WAYPT_PUB = n.advertise<geometry_msgs::Point>("waypoint",1000);
    // Subscriber COSTMAP_SUB=n.subscribe("ransac_to_costmap",1,waypoint_generate);
    // Subscriber sub1 = n.subscribe("/scan", 10, &laserscan);
    // Subscriber sub2 = n.subscribe("/camera/image_raw", 1, &imageCb);

    image_transport::ImageTransport it(n);
    image_transport::Subscriber sub = it.subscribe("/camera/image_raw", 1000, imageCb);

  while(ros::ok())
  {
  
        if(!MALI){
            spinOnce();
            continue;
        }
      Mat roi;

        namedWindow("dj",0);
        imshow("dj",frame_orig);
        // waitKey(0);


        
  /*  if(count>0)
    {
      count--;
      continue;

    }*/
    //frame=object_remove(frame);
    //imshow("win",frame);
        
    
    Mat img1,img2,img3;//temporary images for using filters
    Mat b_processed;
    Mat bg_processed;
    Mat br_processed;
        
     
    //pipeline begins


        
    //extraction of region of interest
        Mat channels[3];
        roi=ROI(frame_orig,0.25,1,0,1); 
        // cout<<"test"<<endl;


        //to plot ransac on a black image
        Mat lanes_by_ransac(roi.rows,roi.cols,CV_8UC3,Scalar(0,0,0));


        /*uses perspective transform,
         homography needs to be changed accordingly
         function is in find_pothole.hpp */
        // cout<<"test"<<endl;
        //removing shadows

        namedWindow("roi",0);
        imshow("roi", roi);
        waitKey(20);
        // roi=removeShadow(roi);
        // spinOnce();
        // return 0;


        obstacle=object_remove(roi);

        namedWindow("as",0);
        imshow("as",obstacle);
        waitKey(10);
        spinOnce(); 
        continue;


        // return 0;

        // cout<<"test"<<endl;
        //object removal

        //bird-eye-view transform
        
        // imshow("win",roi);

        //extraction of different channels
        Mat br=twoblueonered(roi);
    Mat bg=twoblueonegreen(roi);
    Mat b=oneblue(roi);
        
        //processing done for various channels


        //processing for blue channel
        GaussianBlur( b, img1, Size( 9, 9), 0, 0);
        adaptiveThreshold(img1,img2,255,ADAPTIVE_THRESH_MEAN_C,THRESH_BINARY,51,-30);
    medianBlur(img2,b_processed,17);
        
        //processing for 2blue-green channel
        adaptiveThreshold(bg,bg_processed,255,ADAPTIVE_THRESH_MEAN_C,THRESH_BINARY,51,-30);
    medianBlur(bg_processed,bg_processed,13);

        //processing for 2*blue-red chanel
        img1=bg.clone();

    for(int i=1;i<br.rows;i++)
    {
      for(int j=0;j<br.cols;j++)
      {
        if(variance(i,j,&br)>1500)
          img1.at<uchar>(i,j)=0;
      }
    }

    adaptiveThreshold(img1,br_processed,255,ADAPTIVE_THRESH_MEAN_C,THRESH_BINARY,71,-50);
    medianBlur(br_processed,br_processed,11);



        //union of all lane filters
        Mat final_union(roi.rows,roi.cols,CV_8UC1,Scalar(0));
        add_image(final_union, bg_processed,br_processed,b_processed);

        namedWindow("final_union_withobs",0);
        imshow("final_union_withobs",final_union);
        waitKey(10);

        //remove obstacle from processed image
        for(int i=0;i<roi.rows;i++)
        {
            for(int j=0;j<roi.cols;j++)
            {
                if(obstacle.at<Vec3b>(i,j)[0]==255 && obstacle.at<Vec3b>(i,j)[1]==255 && obstacle.at<Vec3b>(i,j)[2]==255)
                {
                    final_union.at<uchar>(i,j)=0;
                }

                // if(pot_hole.at<uchar>(i,j)==255)
                // {
                //     final_union.at<uchar>(i,j)=0;
                // }
            }
        }  

        namedWindow("final_union_withoutobs",0);
        imshow("final_union_withoutobs",final_union);
        waitKey(10);
    }

    return 0;
}