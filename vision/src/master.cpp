#include <iostream>
#include <math.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/mat.hpp>
#include "lane_segmentation.hpp"
//#include "object.hpp"
#include "ransac.hpp"
//#include "mlesac.hpp"
#include "shadowRemoval.hpp"
// #include "beginner_tutorials/model.h"
#include "basic.hpp"
#include "find_pothole.hpp"
#include <geometry_msgs/Point.h>
#include <ros/ros.h>
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/Image.h"
// #include "obsplot.hpp"
#include "cv_bridge/cv_bridge.h"
#include "lidar_new.hpp"
#include "waypts1_header.hpp"
// #include "combinedobs.hpp"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
// #include "object.hpp"
// #include <grid_object.hpp>
// #include "white_obstacle.hpp"

#define ppm 50

using namespace std;
using namespace cv;
using namespace ros;


Publisher ls;
Mat frame_orig;
bool MALI = false;

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
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
    frame_orig = (cv_ptr->image);
    MALI=true;
}

int main(int argc, char **argv)
{ 

    Mat obstacle;
	Mat inflated; 
	model lanes;		//For Ransac implementation(it is a structure)
	lanes.a1=0;lanes.b1=0;lanes.c1=0;lanes.a2=0;lanes.b2=0;lanes.c2=0;
	init(argc,argv,"master");
	NodeHandle n;
    ls = n.advertise<sensor_msgs::LaserScan>("ls", 1000);
   // Publisher RANSAC_PUB= n.advertise<beginner_tutorials::model>("ransac_to_costmap",1000);
    Publisher WAYPT_PUB = n.advertise<geometry_msgs::Point>("waypoint",1000);
    // Subscriber COSTMAP_SUB=n.subscribe("ransac_to_costmap",1,waypoint_generate);
    Subscriber sub1 = n.subscribe("/scan", 10, &laserscan);
	Subscriber sub2 = n.subscribe("/camera/image_raw", 1, &imageCb);
	// Subscriber sub3 = n.subscribe("/camera/image_raw", 1, &obsplotter);
	image_transport::ImageTransport it(n);
    image_transport::Subscriber sub = it.subscribe("/camera/image_raw", 1000, imageCb);
	while(ros::ok())
	{
        if(!MALI)
        {
            spinOnce();
            continue;
        }
        if(!display)
        {
        	spinOnce();
        	continue;
        }

		Mat roi;
        Mat img1,img2,img3;			//temporary images for using filters
		Mat b_processed;
		Mat bg_processed;
		Mat br_processed;
		
		//extraction of region of interest
		roi=ROI(frame_orig,0.25,1,0,1);
		namedWindow("ori",0);
		imshow("ori",roi); 
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
		
		namedWindow("roi",0);
		imshow("roi",roi); 
		waitKey(20);
  		spinOnce();
  		continue;
		//to plot ransac on a black image
        Mat lanes_by_ransac(roi.rows,roi.cols,CV_8UC3,Scalar(0,0,0));
        
        //removing shadows
        // roi=removeShadow(roi);
        // namedWindow("shadowRemoval",0);
		// imshow("shadowRemoval",roi); 
  		//	spinOnce();
  		//	continue;
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
  //       namedWindow("final_union",0);
		// imshow("final_union",final_union); 
  		//	spinOnce();
  		//	continue;
        
        // for(int i=0;i<roi.rows;i++)
        // {
        //     for(int j=0;j<roi.cols;j++)
        //     { 
        //         if(obstacle.at<uchar>(i,j)==255)
        //         {
        //             final_union.at<uchar>(i,j)=0;
        //         }
        //     }
        // }  
        
        final_union=perspective_transform_3channel(frame_orig);
  //       namedWindow("top_view",0);
		// imshow("top_view",final_union); 
  		//	spinOnce();
  		//	continue;
       
        
        // Mat pot_hole=find_pothole(final_union);
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
        //             final_union.at<uchar>(i,j)=0;
        //         }
        //     }
        // } 
 	 	// namedWindow("pot_hole_removed",0);
		// imshow("pot_hole_removed",final_union); 
  		//	spinOnce();
  		//	continue; 

		// curve fitting
        lanes=getRansacModel(final_union,lanes);

        if(lanes.a1==0 && lanes.b1==0 && lanes.c1==0 && lanes.a2!=0 && lanes.b2!=0 && lanes.c2!=0)
        {
            for(int i=0;i<lanes_by_ransac.rows;i++)
            {
                for(int j=0;j<lanes_by_ransac.cols;j++)
                {
                    if(fabs(lanes.a2*i*i+lanes.b2*i+lanes.c2-j)<3)
                    {
                        lanes_by_ransac.at<Vec3b>(i,j)={255,255,255};
                    }
                }
            }   
        }

        else if(lanes.a1!=0 && lanes.b1!=0 && lanes.c1!=0 && lanes.a2==0 && lanes.b2==0 && lanes.c2==0)
        {
            for(int i=0;i<lanes_by_ransac.rows;i++)
            {
                for(int j=0;j<lanes_by_ransac.cols;j++)
                {
                    if(fabs(lanes.a1*i*i+lanes.b1*i+lanes.c1-j)<3) 
                        {
                            lanes_by_ransac.at<Vec3b>(i,j)={255,255,255};
                        }
                }
            }
        }
        
        else
        {
            for(int i=0;i<lanes_by_ransac.rows;i++)
            {
                for(int j=0;j<lanes_by_ransac.cols;j++)
                {
                    if(fabs(lanes.a1*i*i+lanes.b1*i+lanes.c1-j)<3) 
                        {
                            lanes_by_ransac.at<Vec3b>(i,j)={255,255,255};
                        }
                }
            }
            for(int i=0;i<lanes_by_ransac.rows;i++)
            {
                for(int j=0;j<lanes_by_ransac.cols;j++)
                {
                    if(fabs(lanes.a2*i*i+lanes.b2*i+lanes.c2-j)<3)
                    {
                        lanes_by_ransac.at<Vec3b>(i,j)={255,255,255};
                    }
                }
            }
        }
  //       namedWindow("lanes",0);
		// imshow("lanes",lanes_by_ransac); 
  		//	spinOnce();
  		//	continue;
		
		// img_to_ls(lanes_by_ransac);
  		//	spinOnce();
  		//	continue;

        // beginner_tutorials::model msg;
        // msg.numModel = lanes.numModel;
        // msg.a1=lanes.a1;
        // msg.a2=lanes.a2;
        // msg.b1=lanes.b1;
        // msg.b2=lanes.b2;
        // msg.c1=lanes.c1;
        // msg.c2=lanes.c2;
        // publish->lanes to costmap
        // RANSAC_PUB.publish(msg);
        namedWindow("lidar_plot",0);
		imshow("lidar_plot",lidar_plot); 
        inflated=obstaclePlot;
        namedWindow("lidar_inflated_obs",0);
		imshow("lidar_inflated_obs",obstaclePlot); 
  		//	spinOnce();
  		//	continue;
        
		// output waypoint=find_waypoint(lanes,inflated);

  		//	geometry_msgs::Point msg1;
		//	msg1.x=waypoint.x;
  		//	msg1.y=waypoint.y;
  		//	msg1.z=waypoint.angle;

  		// beginner_tutorials::mat msg2;
		// WAYPT_PUB.publish(msg1);
		// subscribe->from costmap

		waitKey(30);
        spinOnce();
	}
	return 0;
}
