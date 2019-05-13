#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include <bits/stdc++.h>
#include "sensor_msgs/NavSatFix.h"
#include <bits/stdc++.h>

using namespace std;
using namespace ros;

#define xstart1 22.3207493  /// seq: 4100
#define ystart1 87.3112742
#define xfinal1 22.3207819  /// seq: 4149
#define yfinal1 87.3113967
#define xstart2 22.3208666  /// seq: 4888
#define ystart2 87.311033
#define xfinal2 22.3208595  /// seq: 4902
#define yfinal2 87.311038
#define radius 0.1
#define pi 3.14159265359

sensor_msgs::NavSatFix coordinates;

double xcurrent,ycurrent;
int seq;

void getgps(sensor_msgs::NavSatFix msg)
{
	coordinates = msg;
	xcurrent = coordinates.latitude;
	ycurrent = coordinates.longitude;
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

	Subscriber sub = n.subscribe("/fix_c", 1000, &getgps);
	Publisher decider_pub = n.advertise<std_msgs::Bool>("/decider", 1000);

	double radius1,radius2,radius3,radius4;

	bool flagstart1 = false,flagfinal1 = false;
	bool flagstart2 = false,flagfinal2 = false;
	std_msgs::Bool decider_msg;
	decider_msg.data = true;
	Rate loop_rate(10);
	while(ok())
	{
		radius1 = distance(xcurrent,ycurrent,xstart1,ystart1);
		radius2 = distance(xcurrent,ycurrent,xfinal1,yfinal1);
		radius3 = distance(xcurrent,ycurrent,xstart2,ystart2);
		radius4 = distance(xcurrent,ycurrent,xfinal2,yfinal2);

		if (!flagstart1 && (radius1 < radius))
		{
			flagstart1 = true;
			decider_msg.data = false;
			//cout << "gps running 1\n";
			//cout << seq << endl;
		}

		if (!flagfinal1 && (radius2 < radius))
		{
			flagfinal1 = true;
			decider_msg.data = true;
			//cout << "lane running 1\n";
			//cout << seq << endl;
		}

		if (!flagstart2 && (radius3 < radius))
		{
			flagstart2 = true;
			decider_msg.data = false;
			//cout << "gps running 2\n";
			//cout << seq << endl;
		}

		if (!flagfinal2 && (radius4 < radius))
		{
			flagfinal2 = true;
			decider_msg.data = true;
			//cout << "lane running 2s\n";
			//cout << seq << endl;
		}

		decider_pub.publish(decider_msg);
		loop_rate.sleep();
		spinOnce();
	}

	return 1;
}
