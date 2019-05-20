#include "lidar_new.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/core/mat.hpp>
#include <iostream>
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

int main(int argc, char **argv)
{
	// cout<<"d"<<endl;
	init(argc,argv,"lidar");
	NodeHandle n;
	Subscriber sub1 = n.subscribe("/scan", 10, &laserscan);

	while(ros::ok())
	{
        if(!display){
            // cout<<"e"<<endl;
            spinOnce();
            continue;
        }
        // cout<<"false"<<endl;
        namedWindow("lidar_new",0);       
        imshow("lidar_new",lidar_plot);
        spinOnce();
    
        waitKey(30);
        // display=false;
    }
        
}