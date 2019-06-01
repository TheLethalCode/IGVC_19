#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include <bits/stdc++.h>
#include "sensor_msgs/NavSatFix.h"
#include <bits/stdc++.h>
#include <gps_waypoint.hpp>

using namespace std;
using namespace ros;
#define radius 1
#define pi 3.14159265359
char* gps_file = "gps_points.txt"; 


sensor_msgs::NavSatFix coordinates;

double current_lat, current_long;
int seq;
bool gps_data = 0;

void getgps(sensor_msgs::NavSatFix msg)
{
	gps_data = 1;
	coordinates = msg;
	current_lat = coordinates.latitude;
	current_long = coordinates.longitude;
	seq = coordinates.header.seq;
}

double distance(double x1,double y1,double x2,double y2)
{
	double R = 6371e3;
	double angle1 = x1 * pi / 180;
	double angle2 = x2 * pi / 180;
	double lat_diff = angle2 - angle1;
	double lon_diff = (y2 - y1) * pi / 180;

	double a = sin(lat_diff/2) * sin(lat_diff/2) + cos(angle1) * cos(angle1) * sin(lon_diff/2) * sin(lon_diff/2);

	double c = 2 * atan2(sqrt(a),sqrt(1-a));

	double d = R * c;

	return d;
}

int main(int argc, char** argv)
{
	init (argc, argv, "switchgps");
	NodeHandle n;

	Subscriber sub = n.subscribe("/gps/filtered", 1, &getgps);
	Publisher use_vision_publisher = n.advertise<std_msgs::Bool>("/use_vision", 1);
	
	//read waypoints
	double start_lat, start_long, end_lat ,end_long;
	FILE* gps_pts;
	try {
		gps_pts=fopen(gps_file,"r");
		if (gps_pts == NULL) {
			throw -1;
		}
		fscanf(gps_pts,"%lf%lf%lf%lf",&start_lat, &start_long, &end_lat, &end_long);
	}
	catch (int e) {
		ROS_INFO("Try running the gps switching code from the root of the IGVC workspace\n");
		return 0;
	}

	double radius1,radius2;

	bool flagstart1 = false,flagfinal1 = false;
	bool flagstart2 = false,flagfinal2 = false;
	
	std_msgs::Bool use_vision_msg;
	use_vision_msg.data = true;
	
	Rate loop_rate(10);
	
	int count = 0, count1=0;
	while(ok())
	{
		if(gps_data == 1)
		{
			radius1 = distance(current_lat, current_long, start_lat, start_long);
			radius2 = distance(current_lat, current_long, end_lat, end_long);
	
			cout<<"radius1= "<<radius1<<"radius2= "<<radius2<<endl;
	
			if (radius1 < radius) {
				count++;
			}
			else {
				count = 0;
			}
	
			if (!flagstart1 && (radius1 < radius) && count >= 10)
			{
				flagstart1 = true;
				use_vision_msg.data = false;
				
				int q = 100;
				while(q >= 0) {
					use_vision_publisher.publish(use_vision_msg);
					q--;
				}
		
				//makes bot reach goal. Will retrun 0 if successful or 1 if not. Wont return until something happens
				int gps_status = gps_waypoint(end_lat, end_long, current_lat, current_long); 
				spinOnce();
	
				radius2 = distance(current_lat, current_long, end_lat, end_long);
				while (radius2 > radius) {
					gps_status = gps_waypoint(end_lat, end_long, current_lat, current_long); 
					radius2 = distance(current_lat, current_long, end_lat, end_long);
					spinOnce();
				}
	
				if (gps_status == 0) {
					use_vision_msg.data = true;
				}
			}
	
			use_vision_publisher.publish(use_vision_msg);
			loop_rate.sleep();
			spinOnce();
		}
		else
		{
			count1++;
			if(count1>10) 
			{
				cout << "No GPS Input" << endl;
				count1=0;
			}
			spinOnce();
			loop_rate.sleep();
			
		}
	}

	return 1;
}
