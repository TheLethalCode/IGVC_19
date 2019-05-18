#ifndef LIDAR
#define LIDAR

#include <bits/stdc++.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/Image.h"

#define obstacleWidth 30
using namespace ros;
using namespace std;
using namespace cv;

Mat imgogi;
Mat lidar_plot;
Mat obstaclePlot;
bool is_laserscan_retrieved = false;

void laserscan(sensor_msgs::LaserScan msg)   /// CALLBACK FOR LIDAR DATA
{
	sensor_msgs::LaserScan scan;
	scan = msg;
	int cols;
	int rows = cols = (scan.range_max * 10);
	Mat img10(rows, cols, CV_8UC1, Scalar(0));
	Mat lidar_plot_ini = img10.clone();

	double minangle = scan.angle_min;
	double step = scan.angle_increment;
	int iters = (scan.angle_max - scan.angle_min) / step;

	for (int i=0;i<=iters;i++)
	{
		double angle = minangle + i*step;
		if (scan.ranges[i] <= scan.range_max)   /// PLOTTING POINTS ACCORDING TO DATA RECIEVED
		{
			float a = scan.ranges[i] * sin(angle);
			float b = scan.ranges[i] * cos(angle);
			a *= 100;
			b *= 100;
			double y = rows/2 + a;
			double x = cols/2 - b;
			if(y>=0 && x<lidar_plot_ini.cols &&x>=0 && y<lidar_plot_ini.rows)
				lidar_plot_ini.at<uchar>(lidar_plot_ini.rows - y, lidar_plot_ini.cols - x) = 255; 
			// int k = (int)(x);
			// int l = (int)(y);
			// for(int g = 0; g < obstacleWidth; ++g)
			// {
			// 	if(((k-g) >= 0 && (k-g) < obstaclePlot.rows) && (l >= 0 && l < obstaclePlot.cols))
			// 	{
			// 		obstaclePlot.at<uchar>(k-g, l) = 255;
			// 	}
			// }
		}
	}
	Point2f pc(lidar_plot_ini.cols/2, lidar_plot_ini.rows/2);
	Mat r = getRotationMatrix2D(pc, 90, 1);
	warpAffine(lidar_plot_ini, lidar_plot, r, lidar_plot_ini.size());
	obstaclePlot=lidar_plot.clone();
	for(int i=0;i<lidar_plot.rows;i++)
		for(int j=0;j<lidar_plot.cols;j++)
			if(lidar_plot.at<uchar>(i,j)==255)
			{
				Point2f center(j,i);
				circle(obstaclePlot, center, 5, Scalar(255), -1);
			}
	is_laserscan_retrieved = true;
}

// void image(sensor_msgs::Image msg)   /// CALLBACK FUNCTION FOR IMAGE DATA
// {
// 	cv_bridge::CvImagePtr cv_ptr;
// 	try
//     {
// 	   cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
//     }
//     catch (cv_bridge::Exception& e)
//     {
//         ROS_ERROR("cv_bridge exception: %s", e.what());
//         return;
//     }

// 	imgogi = cv_ptr->image;
// }

#endif
