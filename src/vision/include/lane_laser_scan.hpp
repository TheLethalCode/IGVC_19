#ifndef LANE_LASER_SCAN
#define LANE_LASER_SCAN

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"
#include <cmath>
#include <limits>
#include <iostream>
using namespace ros;
using namespace std;
using namespace cv;

#define yshift 0.33   /// distance from first view point to lidar in metres
#define angleshift 0.0524    /// angle between camera and lidar axis in radians
#define pixelsPerMetre 137.457    
#define bins 1080  /// no of bins

sensor_msgs::LaserScan laneLaser(Mat img)    /// Input binary image for conversion to laserscan
{
	int row = img.rows;
	int col = img.cols;
	sensor_msgs::LaserScan scan;
	scan.angle_min = -CV_PI/2;
	scan.angle_max = CV_PI/2;
	scan.angle_increment = CV_PI/bins;
	double inf = std::numeric_limits<double>::infinity();
	scan.range_max = inf; 
	
	scan.header.frame_id = "laser";

	for (int i=0;i<bins;i++)
	{
		scan.ranges.push_back(scan.range_max);
	}

	scan.range_max = 80;
	for(int i = 0; i < row; ++i)
	{
		for(int j = 0; j < col; ++j)
		{
			if(img.at<uchar>(i, j) == 255)
			{
				float a = (j - col/2)/pixelsPerMetre;
				float b = (row - i)/pixelsPerMetre + yshift;

				double angle = atan(a/b);

				double r = sqrt(a*a  + b*b);

				int k = (angle - scan.angle_min)/(scan.angle_increment);
				scan.ranges[bins-k-1] = r ;
			}
		}
	}

	return scan;    /// returns Laserscan data
}

#endif