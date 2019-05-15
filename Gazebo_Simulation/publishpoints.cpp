#include "ros/ros.h"
#include <bits/stdc++.h>
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"

using namespace ros;
using namespace std;

nav_msgs::Odometry currpos;
float xcurr,ycurr,zcurr;
bool started = false;

void getpos(nav_msgs::Odometry msg)
{
	currpos = msg;
	xcurr = currpos.pose.pose.position.x;
	ycurr = currpos.pose.pose.position.y;
	zcurr = currpos.pose.pose.position.z;
	// cout << xcurr << " " << ycurr << " " << zcurr <<endl;
}

int main(int argc, char** argv)
{
	init (argc, argv, "publishpoints");

	NodeHandle n;

	Subscriber sub = n.subscribe("/odometry/filtered", 1, &getpos);
	Publisher pub = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 5);

	Rate rate(1);

	ifstream file;
	file.open("/home/kirtan/workspace/src/waypoints/src/waypoints1.txt");

	int seq;
	float radius = 0;
	float xfinal,yfinal,zfinal;
	geometry_msgs::PoseStamped point;

	while(ok())
	{
		radius = sqrt((xcurr-xfinal)*(xcurr-xfinal)+(ycurr-yfinal)*(ycurr-yfinal));
		// cout << radius << endl;
		if (radius < 1.5000)
		{
			float t;
			file >> seq;
			point.header.seq = seq;
			file >> t;
			point.header.stamp = (Time)t;
			file >> point.header.frame_id;
			file >> point.pose.position.x;
			file >> point.pose.position.y;
			file >> point.pose.position.z;
			file >> point.pose.orientation.x;
			file >> point.pose.orientation.y;
			file >> point.pose.orientation.z;
			file >> point.pose.orientation.w;

			cout << point << endl;
			
			xfinal = point.pose.position.x;
			yfinal = point.pose.position.y;
			zfinal = point.pose.position.z;
		}
		pub.publish(point);
		rate.sleep();
		spinOnce();
	}

	return 1;
}