#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include <bits/stdc++.h>
#include "sensor_msgs/NavSatFix.h"
#include <bits/stdc++.h>
#include <gps_waypoint.hpp>

using namespace std;
using namespace ros;
#define radius 3
#define pi 3.14159265359


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
  Publisher pub = n.advertise<move_base_msgs::MoveBaseGoal>("/move_base_simple/goal",2);
  

  //read waypoints
  double start_lat, start_long, mid1_lat ,mid1_long, mid2_lat ,mid2_long, end_lat, end_long;
  FILE* gps_pts;
  try {
    gps_pts=fopen("gps_points.txt","r");
    if (gps_pts == NULL) {
      throw -1;
    }
    fscanf(gps_pts,"%lf %lf %lf %lf",&start_lat, &start_long, &mid1_lat, &mid1_long);
    fscanf(gps_pts,"%lf %lf %lf %lf",&mid2_lat, &mid2_long, &end_lat, &end_long);
  }
  catch (int e) {
    ROS_INFO("Try running the gps switching code from the root of the IGVC workspace\n");
    return 0;
  }

  double radius1,radius2, radius3, radius4;

  bool flagstart1 = false;

  std_msgs::Bool use_vision_msg;
  use_vision_msg.data = true;

  Rate loop_rate(10);

  int count1 = 0, count_no_gps=0;
  while(ok())
  {
    if(gps_data == 1)
    {
      radius1 = distance(current_lat, current_long, start_lat, start_long);
      radius2 = distance(current_lat, current_long, mid1_lat, mid1_long);
      radius3 = distance(current_lat, current_long, mid2_lat, mid2_long);
      radius4 = distance(current_lat, current_long, end_lat, end_long);

      cout<< "radius1: " << radius1 << endl;
      cout<< "radius2: " << radius2 << endl;
      cout<< "radius3: " << radius3 << endl;
      cout<< "radius4: " << radius4 << endl;


      if (radius1 < radius) {
        count1++;
      }
      else {
        count1 = 0;
      }


      //entering no man's land
      if (!flagstart1 && (radius1 < radius) && count1 >= 10)
      {
	      move_base_msgs::MoveBaseGoal gps_status;
        flagstart1 = true;
        use_vision_msg.data = false;

        int q = 100;
        while(q >= 0) {
          use_vision_publisher.publish(use_vision_msg);
          q--;
        }


        //change teb parameters
        cout << "running no man's land bash file" << endl;
        system("bash teb_params_no_mans_land.sh");

        spinOnce();
        radius2 = distance(current_lat, current_long, mid1_lat, mid1_long);
        spinOnce();

        while (radius2 > radius) {
          gps_status = gps_waypoint(mid1_lat, mid1_long, current_lat, current_long); 
          pub.publish(gps_status);
          //makes bot reach goal. Will retrun 0 if successful or 1 if not. Wont return until something happens
          radius2 = distance(current_lat, current_long, mid1_lat, mid1_long);
          spinOnce();
        }

        spinOnce();
        radius3 = distance(current_lat, current_long, mid2_lat, mid2_long);
        spinOnce();

        while (radius3 > radius) {
          gps_status = gps_waypoint(mid2_lat, mid2_long, current_lat, current_long); 
          pub.publish(gps_status);

          radius3 = distance(current_lat, current_long, mid2_lat, mid2_long);
          spinOnce();
        }

        spinOnce();
        radius4 = distance(current_lat, current_long, end_lat, end_long);
        spinOnce();

        while (radius4 > radius) {
          gps_status = gps_waypoint(end_lat, end_long, current_lat, current_long); 
          pub.publish(gps_status);

          radius4 = distance(current_lat, current_long, end_lat, end_long);
          spinOnce();
        }

        
        use_vision_msg.data = true;
        cout << "running vision bash file" << endl;
        system("bash teb_params_vision.sh");
        
      }

      use_vision_publisher.publish(use_vision_msg);
      loop_rate.sleep();
      gps_data = 0;
      spinOnce();
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
      loop_rate.sleep();

    }
  }

  return 1;
}
