#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include <bits/stdc++.h>
#include "sensor_msgs/NavSatFix.h"
#include <bits/stdc++.h>
#include <gps_waypoint.hpp>

using namespace std;
using namespace ros;
/*
#define xstart1_lat 22.3210062 /// seq: 4100
#define ystart1_long 87.3111907
#define xfinal1_lat 22.3208642 /// seq: 4149
#define yfinal1_long 87.3110999*/
// #define xstart2_lat 22.3208666  /// seq: 4888
// #define ystart2_long 87.311033
// #define xfinal2_lat 22.3208595  /// seq: 4902
// #define yfinal2_long 87.311038
#define radius 1
#define pi 3.14159265359

sensor_msgs::NavSatFix coordinates;

double xcurrent_lat,ycurrent_long;
int seq;

void getgps(sensor_msgs::NavSatFix msg)
{
	coordinates = msg;
	xcurrent_lat = coordinates.latitude;
	ycurrent_long = coordinates.longitude;
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
	double xstart1_lat,ystart1_long,xfinal1_lat,yfinal1_long;
	FILE* gps_pts;
	gps_pts=fopen("gps_points.txt","r");
	fscanf(gps_pts,"%lf%lf%lf%lf",&xstart1_lat,&ystart1_long,&xfinal1_lat,&yfinal1_long);

	
	init (argc, argv, "switchgps");
    ros::param::get("/outdoor_waypoint_nav/coordinates_file", path_local);
    //numWaypoints = countWaypointsInFile(path_local);

    //Reading waypoints from text file and output results
    //waypointVect = getWaypoints(path_local);

	NodeHandle n;

	Subscriber sub = n.subscribe("/gps/filtered", 1, &getgps);
	Publisher use_vision_publisher = n.advertise<std_msgs::Bool>("/use_vision", 1);

	double radius1,radius2,radius3,radius4;

	bool flagstart1 = false,flagfinal1 = false;
	bool flagstart2 = false,flagfinal2 = false;
	std_msgs::Bool use_vision_msg;
	use_vision_msg.data = true;
	Rate loop_rate(10);
	int count = 0;
	while(ok())
	{
		//cout<<"in loop"<<endl;
		radius1 = distance(xcurrent_lat,ycurrent_long,xstart1_lat,ystart1_long);
		radius2 = distance(xcurrent_lat,ycurrent_long,xfinal1_lat,yfinal1_long);

		cout<<"raduis1="<<radius1<<"  radius2  "<<radius2<<endl;
		// radius2 = distance(xcurrent,ycurrent,xfinal1,yfinal1);
		// radius3 = distance(xcurrent,ycurrent,xstart2,ystart2);
		// radius4 = distance(xcurrent,ycurrent,xfinal2,yfinal2);

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
	
			int gps_status = gps_waypoint(xfinal1_lat, yfinal1_long);

			spinOnce();
			radius2 = distance(xcurrent_lat,ycurrent_long,xfinal1_lat,yfinal1_long);
			while (radius2 > radius) {
				gps_status = gps_waypoint(xfinal1_lat, yfinal1_long);
				radius2 = distance(xcurrent_lat,ycurrent_long,xfinal1_lat,yfinal1_long);
				spinOnce();
			}

			if (gps_status == 0) {
				use_vision_msg.data = true;
			}
			//cout << "gps running 1\n";
			//cout << seq << endl;
		}

		// if (!flagfinal1 && (radius2 < radius))
		// {
		// 	flagfinal1 = true;
		// 	use_vision_msg.data = true;
		// 	//cout << "lane running 1\n";
		// 	//cout << seq << endl;
		// }

		// if (!flagstart2 && (radius3 < radius))
		// {
		// 	flagstart2 = true;
		// 	use_vision_msg.data = false;
		// 	//cout << "gps running 2\n";
		// 	//cout << seq << endl;
		// }

		// if (!flagfinal2 && (radius4 < radius))
		// {
		// 	flagfinal2 = true;
		// 	use_vision_msg.data = true;
		// 	//cout << "lane running 2s\n";
		// 	//cout << seq << endl;
		// }

		use_vision_publisher.publish(use_vision_msg);
		loop_rate.sleep();
		spinOnce();
	}

	return 1;
}
