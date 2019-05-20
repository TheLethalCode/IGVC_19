#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/Image.h"
#include "cv_bridge/cv_bridge.h"

include the above headers in the code 

The header Lidar_new.hpp contains two callback functions for subscribers of image data and lidar data

init (argc, argv, "tester");       ///  initialize ros

NodeHandle n;        /// create node handle

Subscriber sub1 = n.subscribe("/scan", 10, &laserscan);        /// subscibes laser data

Subscriber sub2 = n.subscribe("/camera/image_color", 10, &image);        /// subscibes image data

finally to show lidar data   use the variable (lidar_plot) which is globally declared in header
and use   (imgogi) for showing image data



//////////   EXAMPLE CODE OF MAIN

int main(int argc, char **argv)
{
	init (argc, argv, "tester");
	
	NodeHandle n;

	Subscriber sub1 = n.subscribe("/scan", 10, &laserscan);
	Subscriber sub2 = n.subscribe("/camera/image_color", 10, &image);

	while (ok())
	{
		if (display)      /// FOR HANDLING ERROR
		{
			imshow("1", lidar_plot);
			waitKey(1);
		}

		spinOnce();
	}
	return 0;
}