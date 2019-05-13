#ifndef LIDAR
#define LIDAR

#include <bits/stdc++.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"

using namespace ros;
using namespace std;
using namespace cv;

Mat imgogi;
Mat lidar_plot;
// bool display = false;

void laserscan(sensor_msgs::LaserScan msg)   /// CALLBACK FOR LIDAR DATA
{
	sensor_msgs::LaserScan scan;
	scan = msg;
	int cols;
	int rows = cols = (scan.range_max * 10);
	Mat img10(rows, cols, CV_8UC1, Scalar(0));
	lidar_plot = img10.clone();

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
			a *= 10;
			b *= 10;
			double y = rows/2 + a;
			double x = cols/2 - b;
			
			lidar_plot.at<uchar>(x, y) = 255; 
		}
	}
	display = true;
}

void image(sensor_msgs::Image msg)   /// CALLBACK FUNCTION FOR IMAGE DATA
{
	cv_bridge::CvImagePtr cv_ptr;
	try
    {
	   cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

	imgogi = cv_ptr->image;
}

#endif
