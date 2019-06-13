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


/*
   Converts any binary image's white points(Obstacles/Lanes) 
   into a LaserScan(LIDAR format) Message.
 */

//Takes a b/w image as input
sensor_msgs::LaserScan laneLaser(Mat img)
{

    spinOnce();
    sensor_msgs::LaserScan scan;
    scan.angle_min = lidar_scan.angle_min;
    scan.angle_max = lidar_scan.angle_max;
    scan.angle_increment = lidar_scan.angle_increment;

    //NOTE the declaration of infinity
    double inf = std::numeric_limits<double>::infinity();
    scan.range_max = lidar_scan.range_max; 
    //range_max= maximum r value after which obstacles are to be seen 

    //NOTE frame_id
    scan.header.frame_id = "laser";
    scan.header.stamp = ros::Time::now();
    scan.scan_time = lidar_scan.scan_time;     //Total scan duration
    scan.time_increment = lidar_scan.time_increment; //Time interval between scanning 2 angles

    //Initialising each LIDAR scan distance to infinity
    //  This allows for rest of the points to not be given in the LaserScan
    for (double angle=scan.angle_min; angle < scan.angle_max; angle += scan.angle_increment)
    {
        scan.ranges.push_back(scan.range_max);
    }

    //Putting in 
    for(int i = 0; i < img.rows; ++i)
    {
        for(int j = 0; j < img.cols; ++j)
        {
            if(img.at<uchar>(i,j)>0)
            {
    
                float a = (img.cols/2 - j)/pixelsPerMetre;
                float b = (img.rows - i)/pixelsPerMetre + yshift;

                double angle = atan(a/b);

                double r = sqrt(a*a  + b*b);

                int k = (angle - scan.angle_min)/(scan.angle_increment);    
                //For finding bin no. 

                //Assigning r if present
                if (r < scan.ranges[k]) {  //NOTE (bins-k-1) 
                    scan.ranges[k] = r;
                }

            }
        }
    }

    for (double angle=scan.angle_min; angle < scan.angle_max; angle += scan.angle_increment)
    {
        int k = (double)(angle - scan.angle_min)/scan.angle_increment;

        if (lidar_scan.ranges[k] < scan.ranges[k]) {
            scan.ranges[k] = lidar_scan.ranges[k];
        }

    }
    return scan;    /// returns LaserScan data
}

#endif
