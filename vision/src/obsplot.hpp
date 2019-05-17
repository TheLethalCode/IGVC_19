#ifndef OBSPLOT
#define OBSPLOT

#include <bits/stdc++.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"

#define obstacleWidth 30

using namespace ros;
using namespace std;
using namespace cv;

Mat obstaclePlot;
bool display1 = false;

void obsplotter(sensor_msgs::LaserScan msg)   /// CALLBACK FOR LIDAR DATA
{
	sensor_msgs::LaserScan scan;
	scan = msg;
	int cols;
	int rows = cols = (scan.range_max * 10);
	Mat img10(rows, cols, CV_8UC1, Scalar(0));
	obstaclePlot = img10.clone();

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

			int k = (int)(x);
			int l = (int)(y);
			for(int g = 0; g < obstacleWidth; ++g)
			{
				if(((k-g) >= 0 && (k-g) < obstaclePlot.rows) && (l >= 0 && l < obstaclePlot.cols))
				{
					obstaclePlot.at<uchar>(k-g, l) = 255;
				}
			}
		}
	}
	display1 = true;
}

#endif