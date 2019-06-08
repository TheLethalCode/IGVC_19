/*#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include <bits/stdc++.h>
#include "sensor_msgs/NavSatFix.h"
#include <bits/stdc++.h>
#include <switchgps.hpp>


using namespace std;
using namespace ros;
#define radius 0.5
#define pi 3.14159265359

int count_no_gps=0;
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
  init (argc, argv, "qualification");
  NodeHandle n;

  Subscriber sub = n.subscribe("/gps/filtered", 1, &getgps);

  //read waypoints
  double goal_lat, goal_long;
  FILE* gps_pts;
  try {
    gps_pts=fopen("gps_qualification_point.txt","r");
    if (gps_pts == NULL) {
      throw -1;
    }
    fscanf(gps_pts,"%lf %lf",&goal_lat, &goal_long);
  }
  catch (int e) {
    ROS_INFO("Try running the gps switching code from the root of the IGVC workspace\n");
    return 0;
  }

  double radius1;

  Rate loop_rate(10);

  int count1 = 0;
  while(ok())
  {
    if(gps_data == 1)
    {

      int gps_status;

      radius1 = distance(current_lat, current_long, goal_lat, goal_long);
      cout<< "radius1: " << radius1 << endl;

      spinOnce();
      int count2 = 0;

      while ((radius1 > radius && count2 <= 10)&& ros::ok()) {


        for (int j = 0; j <= 10; j++) {
          cout << "Still searching for waypoint, radius1: " << radius1 << endl;
        }
        gps_status = gps_waypoint(goal_lat, goal_long, current_lat, current_long); 
        //makes bot reach goal. Will retrun 0 if successful or 1 if not. Wont return until something happens
        radius1 = distance(current_lat, current_long, goal_lat, goal_long);
        if (radius1 < radius) {
          count2++;
        }
        else {
          count2 = 0;
        }
        spinOnce();
      }
      radius1 = distance(current_lat, current_long, goal_lat, goal_long);
      for (int j = 0; j <= 10; j++) {
          cout << "Reached waypoint, radius1: " << radius1 << endl;
      }
      return 0;
    }

    else
    {
      count_no_gps++;
      if(count_no_gps>10) 
      {
        cout << "No GPS Input" << endl;
        count_no_gps=0;
      }
      spinOnce();

    }
    loop_rate.sleep();
  }

  return 1;
}
*/
