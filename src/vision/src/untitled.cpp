#include "lidar_new.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/core/mat.hpp>
#include <iostream>

int main(int argc, char **argv)
{
	init(argc,argv,"master");
	NodeHandle n;
	Subscriber sub1 = n.subscribe("/scan", 10, &laserscan);

	while(ros::ok())
	{
        if(!display){
            spinOnce();
            continue;
        }
        namedWindow("lidar_new",0);       
        imshow("lidar_new",lidar_plot);
        waitKey(30);
        spinOnce();
    }
}