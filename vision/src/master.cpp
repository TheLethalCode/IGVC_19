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
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/Image.h"
#include "obsplot.hpp"
#include "cv_bridge/cv_bridge.h"
#include "lidar_new.hpp"
#include "waypts1_header.hpp"
#include "combinedobs.hpp"

#define ppm 150
using namespace std;
using namespace cv;
using namespace ros;


NodeHandle n;

void img_to_ls(Mat img )
{
    Publisher ls = n.advertise<sensor_msgs::LaserScan>("ls", 1000);
    sensor_msgs::LaserScan msg;
    msg.angle_min=-CV_PI/2;
    msg.angle_max=CV_PI/2;
    int k=0;
    msg.angle_increment=CV_PI/180;
    for(float theta=msg.angle_min;theta<=msg.angle_max;theta+=msg.angle_increment)
        {
            
            for(float r=msg.range_min;r<=msg.range_max;r+=1/ppm)
            {
                int i=img.rows-r*cos(theta)*ppm;
                int j=img.cols/2-r*sin(theta)*ppm;
                if(img.at<uchar>(i,j)==255)
                {
                    msg.ranges[k++]=r;
                    break;
                }
            }
            
        }
    ls.publish(msg);
}

int main(int argc, char **argv)
{
	// VideoCapture vid("vid.mp4",0);  
cout<<"a";
	// Mat frame;
    int i,j;
    Mat obstacle;
	Mat inflated; 
	//int count=500;//frames to be skipped initially
	model lanes;//For Ransac implementation(it is a structure)
	lanes.a1=0;lanes.b1=0;lanes.c1=0;lanes.a2=0;lanes.b2=0;lanes.c2=0;


    init(argc,argv,"master");
   
    // Publisher RANSAC_PUB= n.advertise<beginner_tutorials::model>("ransac_to_costmap",1000);
    Publisher WAYPT_PUB = n.advertise<geometry_msgs::Point>("waypoint",1000);
    // Subscriber COSTMAP_SUB=n.subscribe("ransac_to_costmap",1,waypoint_generate);
    Subscriber sub1 = n.subscribe("/scan", 10, &laserscan);
	Subscriber sub2 = n.subscribe("/camera/image_color", 10, &image);

	while(ok())
	{
		// vid>>frame;	
		Mat roi;
        
	/*	if(count>0)
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
        roi=ROI(imgogi,0,1,0,1); 
        // cout<<"test"<<endl;

        //to plot ransac on a black image
        Mat lanes_by_ransac(roi.rows,roi.cols,CV_8UC3,Scalar(0,0,0));
          

        /*uses perspective transform,
         homography needs to be changed accordingly
         function is in find_pothole.hpp */
        // cout<<"test"<<endl;
        //removing shadows
        roi=removeShadow(roi);

        obstacle=object_remove(roi);

        roi=perspective_transform_3channel(roi);/*homography needs to be changed 
                                        function is in basic.hpp */
        

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
        
        Mat pot_hole=find_pothole(final_union);

        img_to_ls(pot_hole);

        for(i=0;i<roi.rows;i++)
        {
            for(j=0;j<roi.cols;j++)
            {
                if(obstacle.at<uchar>(i,j)==255)
                {
                    final_union.at<uchar>(i,j)=0;
                }

                if(pot_hole.at<uchar>(i,j)==255)
                {
                    final_union.at<uchar>(i,j)=0;
                }
            }
        }





        // curve fitting
        lanes=getRansacModel(final_union,lanes);

        if(lanes.a1==0 && lanes.b1==0 && lanes.c1==0 && lanes.a2!=0 && lanes.b2!=0 && lanes.c2!=0)
        {
            for(i=0;i<lanes_by_ransac.rows;i++)
            {
                for(j=0;j<lanes_by_ransac.cols;j++)
                {
                    if(fabs(lanes.a2*i*i+lanes.b2*i+lanes.c2-j)<3)
                    {
                        lanes_by_ransac.at<Vec3b>(i,j)[0]=255;
                        lanes_by_ransac.at<Vec3b>(i,j)[1]=255;
                        lanes_by_ransac.at<Vec3b>(i,j)[2]=255;
                    }
                }
            }   
        }

        else if(lanes.a1!=0 && lanes.b1!=0 && lanes.c1!=0 && lanes.a2==0 && lanes.b2==0 && lanes.c2==0)
        {
            for(i=0;i<lanes_by_ransac.rows;i++)
            {
                for(j=0;j<lanes_by_ransac.cols;j++)
                {
                    if(fabs(lanes.a1*i*i+lanes.b1*i+lanes.c1-j)<3) 
                        {
                            lanes_by_ransac.at<Vec3b>(i,j)[0]=255;
                            lanes_by_ransac.at<Vec3b>(i,j)[1]=255;
                            lanes_by_ransac.at<Vec3b>(i,j)[2]=255;
                        }
                }
            }
        }
        
        else
        {
            for(i=0;i<lanes_by_ransac.rows;i++)
            {
                for(j=0;j<lanes_by_ransac.cols;j++)
                {
                    if(fabs(lanes.a1*i*i+lanes.b1*i+lanes.c1-j)<3) 
                        {
                            lanes_by_ransac.at<Vec3b>(i,j)[0]=255;
                            lanes_by_ransac.at<Vec3b>(i,j)[1]=255;
                            lanes_by_ransac.at<Vec3b>(i,j)[2]=255;
                        }
                }
            }
            for(i=0;i<lanes_by_ransac.rows;i++)
            {
                for(j=0;j<lanes_by_ransac.cols;j++)
                {
                    if(fabs(lanes.a2*i*i+lanes.b2*i+lanes.c2-j)<3)
                    {
                        lanes_by_ransac.at<Vec3b>(i,j)[0]=255;
                        lanes_by_ransac.at<Vec3b>(i,j)[1]=255;
                        lanes_by_ransac.at<Vec3b>(i,j)[2]=255;
                    }
                }
            }
        }

        img_to_ls(lanes_by_ransac);

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

        inflated=obstaclePlot;

        output waypoint=find_waypoint(lanes,inflated);

        geometry_msgs::Point msg1;

  		msg1.x=waypoint.x;
  		msg1.y=waypoint.y;
  		msg1.z=waypoint.angle;

  		// beginner_tutorials::mat msg2;

        WAYPT_PUB.publish(msg1);





        // subscribe->from costmap

        //publish waypoint to planning 

        //Planning walon aajao


        // imshow("Final image",final_union);

        //imshow("roi",lanes);
  

		// waitKey(1);
        spinOnce();
	}
	return 0;
}
