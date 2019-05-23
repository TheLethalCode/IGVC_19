#include "ros/ros.h"
#include "rosgraph_msgs/Log.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/Imu.h"
#include "std_msgs/String.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/Image.h"
#include <string.h>
#include "sensor_status/sensor_status.h"
#include <sys/time.h>
#include <visualization_msgs/Marker.h> 
#include <tf/transform_listener.h>
using namespace std;

#define sensor_timeout 1 //second

bool flid, fimu, fcam, fgps;
bool blid=0, bimu=0, bcam=0, bgps=0;

timeval tlid1, tcam1, timu1, tgps1;
timeval tlid2, tcam2, timu2, tgps2;

void lidarcb(sensor_msgs::LaserScan scan)
{
    gettimeofday(&tlid2, 0);
	if( scan.ranges.size()>0 || fabs(scan.range_max)>0 ) flid=1;
	else flid=0;
}

void imucb(sensor_msgs::Imu mimu)
{
    gettimeofday(&timu2, 0);
	if(mimu.linear_acceleration.z < -5) fimu=1;
	else fimu=0;
}

void cameracb(sensor_msgs::Image mcam)
{
    gettimeofday(&tcam2, 0);
	if(mcam.data.size()>0 || mcam.width>0 ) fcam=1;
	else fcam=0;
}

void gpscb(sensor_msgs::NavSatFix mgps)
{
    gettimeofday(&tgps2, 0);
	if(mgps.latitude>1) fgps=1;
	else fgps=0;
}

double diff_s(timeval t1, timeval t2)
{
    double a=(((t1.tv_sec - t2.tv_sec) * 1000000) + (t1.tv_usec - t2.tv_usec))/1000;
    return a/1000.0;
}


void vis_sensor_status(ros::Publisher vis_ss, bool blid, bool bgps, bool bcam, bool bimu)
{

	visualization_msgs::Marker ans;
	ans.header.frame_id = "base_link";     
	ans.header.stamp = ros::Time::now();  
	ans.ns = "points_and_lines";     
	ans.action =visualization_msgs::Marker::ADD;     
	ans.pose.orientation.w =  1.0;     
	ans.type = visualization_msgs::Marker::POINTS;  
	ans.id = 0;   
	ans.scale.x = 0.4;     
	ans.scale.y = 0.4; 
	ans.color.a = 1.0; 
 	
 	bool b=blid && bgps && bcam && bimu;
	if(b==1)
	{
		ans.color.r = 0.0;       
	  	ans.color.g = 1.0;   
	}  
	else
	{
		ans.color.r = 1.0;       
	  	ans.color.g = 0.0;   
	}

	geometry_msgs::Point circ;
	circ.x=  0;
	circ.y=  0;
	circ.z=0;
	ans.points.push_back(circ);
	vis_ss.publish(ans);
	sleep(0.01);


	return;
}
int main(int argc, char **argv)
{
	ros::init(argc, argv, "error_log");
	ros::NodeHandle nh;
	ros::Subscriber lidar = nh.subscribe<sensor_msgs::LaserScan>("scan", 1, &lidarcb);
	ros::Subscriber imu = nh.subscribe<sensor_msgs::Imu>("/vn_ins/imu", 1, &imucb);
	ros::Subscriber gps = nh.subscribe<sensor_msgs::NavSatFix>("/vn_ins/fix", 1, &gpscb);
	ros::Subscriber camera = nh.subscribe<sensor_msgs::Image>("/camera/image_color", 1, &cameracb);

	ros::Publisher ss = nh.advertise<sensor_status::sensor_status>("/sensor_status", 1);
    ros::Publisher vis_ss= nh.advertise<visualization_msgs::Marker>("/sensors", 10); 
	
	timeval t1, t0; 
	gettimeofday(&t1, 0);
	ros::Rate loop_rate(20);
	while(ros::ok())
	{
		gettimeofday(&t0, 0);

			if(diff_s(t0,tlid2)< sensor_timeout && flid==1) blid =true;
			else blid=false;
			
			if(diff_s(t0,timu2)< sensor_timeout && fimu==1) bimu =true;
			else bimu=false;
			
			if(diff_s(t0,tcam2)< sensor_timeout && fcam==1) bcam =true;
			else bcam=false;

			if(diff_s(t0,tgps2)< sensor_timeout && fgps==1) bgps =true;
			else bgps=false;
			
			if(blid==1) nh.setParam("lid", 1);
			else nh.setParam("lid", 0);

			if(bcam==1) nh.setParam("cam", 1);
			else nh.setParam("cam", 0);

			if(bimu==1) nh.setParam("imu", 1);
			else nh.setParam("imu", 0);
		if(diff_s(t0,t1)>1)
		{

			sensor_status::sensor_status ans;
			ans.Lidar= blid;
			ans.GPS= bgps;
			ans.Camera= bcam;
			ans.Imu= bimu;
			vis_sensor_status(vis_ss, blid, bgps, bcam, bimu);
			ss.publish(ans);
			gettimeofday(&t1, 0);
		} 
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
