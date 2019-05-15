#include "ros/ros.h"
#include <bits/stdc++.h>
#include "geometry_msgs/PoseStamped.h"

using namespace ros;
using namespace std;

geometry_msgs::PoseStamped point;

void getpoints(geometry_msgs::PoseStamped msg)
{
	point = msg;
}

int main(int argc, char ** argv)
{
	init (argc, argv, "save_waypoints");

	NodeHandle n;

	Subscriber sub = n.subscribe("/move_base_simple/goal", 1000, &getpoints);

	int count = 0;
	ofstream file;
	file.open("/home/kirtan/workspace/src/waypoints/src/waypoints.txt");

	if (file == NULL)
	{
		cout << "no file chutiya kat gaya\n";
	}

	while(ok())
	{
		if (count != point.header.seq)
		{
			cout << point << endl;
			file << point.header.seq << endl;
			file << point.header.stamp << endl; 
			file << point.header.frame_id << endl;
			file << point.pose.position.x << endl;
			file << point.pose.position.y << endl;
			file << point.pose.position.z << endl;
			file << point.pose.orientation.x << endl;
			file << point.pose.orientation.y << endl;
			file << point.pose.orientation.z << endl;
			file << point.pose.orientation.w << endl;
			count = point.header.seq;
		}
		spinOnce();
	}

	return 1;
}