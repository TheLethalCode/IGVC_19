#ifndef LIDAR
#define LIDAR

#include <bits/stdc++.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"

using namespace ros;
using namespace std;
using namespace cv;

sensor_msgs::LaserScan scan;
Mat imgogi;
bool display = false;

void laserscan(sensor_msgs::LaserScan msg)   /// CALLBACK FOR LIDAR DATA
{
	scan = msg;
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
	display = true;
}

void Laser_Data()    /// MAIN FUNCTION TO CALL  
{
	NodeHandle n;

	/// HOMOGRAPHY MATRIX FOR CONVERSION FROM FRONT_VIEW TO TOP_VIEW
	Mat h = (Mat_<double>(3,3) << -6.33180939e-01 ,-8.00968486e-01 ,8.78664976e+02 ,2.15243163e-02 ,-3.81716782e+00 ,2.40813147e+03 ,5.67776805e-05 ,-2.79605922e-03 ,1.00000000e+00);
	int k = 0;
	
	Subscriber sub1 = n.subscribe("/scan", 10, &laserscan);
	Subscriber sub2 = n.subscribe("/camera/image_color", 10, &image);


	namedWindow("2", 0);
	namedWindow("1", 0);
	namedWindow("3", 0);
	
	Mat img10;

	Mat top(1200, 1200, CV_8UC1, Scalar(0));


	while (ok())
	{
		if (scan.angle_min != 0)
		{
			int cols;
			int rows = cols = (scan.range_max * 10);
			Mat img(rows, cols, CV_8UC1, Scalar(0));
			Mat img10(rows, cols, CV_8UC1, Scalar(0));
			
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
					
					img.at<uchar>(x, y) = 255; 
				}
			}
			warpPerspective(img, top, h , top.size(), WARP_INVERSE_MAP);  /// TRANSFORMATION
			imshow("2", top);
			waitKey(1);
			imshow("3", img);
			waitKey(1);
		}

		if (display)   /// DISPLAYING CORRESPONDING IMAGE 
		{
			for(int i = 740; i > 641; i--)
			{
				for(int j = 810; j < 990; j++)
				{
					for(int k = 0; k < 10; k++)
					{
						if(top.at<uchar>(i, j) == 255)
						{
							imgogi.at<Vec3b>(10*(i - 641) + k, 10*(j - 810) + k)[0] = 0;
							imgogi.at<Vec3b>(10*(i - 641) + k, 10*(j - 810) + k)[1] = 0;
							imgogi.at<Vec3b>(10*(i - 641) + k, 10*(j - 810) + k)[2] = 255;
						}
					}
				}
			}
			imshow("1", imgogi);
			waitKey(1);
			display = true;
		}
		spinOnce();
	}
}

#endif

