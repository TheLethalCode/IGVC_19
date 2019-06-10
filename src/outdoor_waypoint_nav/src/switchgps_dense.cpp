#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include <bits/stdc++.h>
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/Imu.h"
#include <bits/stdc++.h>
#include <switchgps.hpp>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

using namespace std;
using namespace ros;
#define radius 1
#define pi 3.14159265359

//for dense obstacles 
double dense_stop_lat ,dense_stop_long;
double dense_start_lat=42.6787258416;
double dense_start_long=-83.1954428839;
double radius1_dense = 3; //in metres
double radius2_dense = 3; //in metres
double angle_enu_dense = 0; //in ENU (degrees)
double distance_1_and_2_dense = 5;


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

int count_odom = 0;

void getodom(sensor_msgs::Imu msg)
{
  tf::Quaternion quat;
  tf::quaternionMsgToTF(msg.orientation, quat);

  double roll, pitch, yaw;
  tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

  if (count_odom < 50) {
    count_odom++;
    orientation_init += yaw;
  }
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
    Subscriber sub_odom = n.subscribe("/imu", 1, &getodom);
    Publisher use_vision_publisher = n.advertise<std_msgs::Bool>("/use_vision", 1);

    //read waypoints
    double start_lat, start_long, mid1_lat ,mid1_long, mid2_lat ,mid2_long, end_lat, end_long;

    FILE* gps_pts;
    try {
        gps_pts=fopen("gps_points.txt","r");
        if (gps_pts == NULL) {
            throw -1;
        }
        // fscanf(gps_pts,"%lf %lf",&dense_start_lat, &dense_start_long;
        fscanf(gps_pts,"%lf %lf %lf %lf",&start_lat, &start_long, &mid1_lat, &mid1_long);
        fscanf(gps_pts,"%lf %lf %lf %lf",&mid2_lat, &mid2_long, &end_lat, &end_long);
    }
    catch (int e) {
        ROS_INFO("Try running the gps switching code from the root of the IGVC workspace\n");
        return 0;
    }

    FILE* enu_points;
    double two_orient = 0, four_orient = 0, dense_orient =0; //in ENU
    try {
        enu_points=fopen("enu_points.txt","r");
        if (enu_points == NULL) {
            throw -1;
        }
        fscanf(enu_points,"%lf %lf", &two_orient, &four_orient);
    }
    catch (int e) {
        ROS_INFO("Try running the gps switching code from the root of the IGVC workspace\n");
        return 0;
    }

    //get the final dense_stop point
    double d= distance_1_and_2_dense; //in metres
    angle_enu_dense = -angle_enu_dense + 90; //convert to NED
    angle_enu_dense *= (pi/180.0); //in radians
    double R = 6378100;

    dense_stop_lat = asin( sin(dense_start_lat*(pi/180.0))*cos(d/R) + cos(dense_start_lat*(pi/180.0))*sin(d/R)*cos(angle_enu_dense));
    dense_stop_long = dense_start_long*(pi/180.0) + atan2(sin(angle_enu_dense)*sin(d/R)*cos(dense_start_lat*(pi/180.0)),cos(d/R)-sin(dense_start_lat*(pi/180.0))*sin(dense_start_lat));

    dense_stop_lat = (dense_stop_lat/pi)*180.0;//in degree
    dense_stop_long = (dense_stop_long/pi)*180.0;//in degree
    printf("Dense stop lat: %lf long: %lf \n", dense_stop_lat, dense_stop_long) ;

    double radius1,radius2, radius3, radius4, radiusdense1, radiusdense2;

    bool flagstart1 = false, flagstart2 = false, flagdense=false;

    while (count_odom < 50 && ros::ok()) {
      spinOnce();
    }

    orientation_init /= 50;

    std_msgs::Bool use_vision_msg;
    use_vision_msg.data = true;






    Rate loop_rate(10);

    int count1 = 0, count2= 0, count3=0, count_no_gps=0;
    while(ok())
    {
        if(gps_data == 1)
        {
            radius1 = distance(current_lat, current_long, start_lat, start_long);
            radius2 = distance(current_lat, current_long, mid1_lat, mid1_long);
            radius3 = distance(current_lat, current_long, mid2_lat, mid2_long);
            radius4 = distance(current_lat, current_long, end_lat, end_long);

            radiusdense1 = distance(current_lat, current_long, dense_start_lat, dense_start_long);
            radiusdense2 = distance(current_lat, current_long, dense_stop_lat, dense_stop_long);

            cout<< "radiusdense1: " << radiusdense1 << endl;
            cout<< "radiusdense2: " << radiusdense2 << endl;
            cout<< "radius1: " << radius1 << endl;
            cout<< "radius2: " << radius2 << endl;
            cout<< "radius3: " << radius3 << endl;
            cout<< "radius4: " << radius4 << endl;
            cout << "--------------" << endl;

            if (radius1 < radius) {
                count1++;
            }
            else {
                count1 = 0;
            }

            if (radius3 < radius) {
                count2++;
            }
            else {
                count2 = 0;
            }
            if (radiusdense1 < radius1_dense) {
                count3++;
            }
            else {
                count3 = 0;
            }
            spinOnce();
            /*---------------------------------------DENSE SWITCH-----------------------------------*/
            //entering no man's land 1st part
            if (!flagdense && count3 >= 10)
            {
                int gps_status;
                flagdense = true;
                /*-------------------------------------1nd Dense waypoint entered-----------------------------------------*/
                for (int j = 0; j <= 10; j++) {
                    cout << "Reached Dense 1st Waypoint" << endl;
                }
                //change teb parameters
                // system("bash teb_params_no_mans_land.sh");

                for (int j = 0; j <= 10; j++) {
                    cout << "Going towards Dense 2nd waypoint" << endl;
                }
                spinOnce();

                
                /*-------------------------------------2nd Dense waypoint-----------------------------------------*/
                radiusdense2 = distance(current_lat, current_long, dense_stop_lat, dense_stop_long);
                spinOnce();
                int count2 = 0;
                while ((radiusdense2 > radius2_dense && count2 <= 10)&& ros::ok()) {
                    for (int j = 0; j <= 10; j++) {
                        cout << "Still searching for Dense 2nd waypoint, radiusdense2: " << radiusdense2 << endl;
                    }
                    gps_status = gps_waypoint(dense_stop_lat, dense_stop_long, current_lat, current_long, angle_enu_dense); 
                    radiusdense2 = distance(current_lat, current_long, dense_stop_lat, dense_stop_long);
                    if (radiusdense2 < radius2_dense) {
                        count2++;
                    }
                    else {
                        count2 = 0;
                    }
                    spinOnce();
                }

                for (int j = 0; j <= 10; j++) {
                    cout << "Switching to vision after Dense 2nd wpt" << endl;
                }
                // system("bash teb_params_vision_middle.sh");
                use_vision_msg.data = true;
                for(int j=0; j<100; j++) {
                    use_vision_publisher.publish(use_vision_msg);
                }
            }

            /*---------------------------------------NO Mans Land Part 1-----------------------------------*/
            //entering no man's land 1st part
            if (!flagstart1 && count1 >= 10)
            {
                int gps_status;
                flagstart1 = true;
                /*-------------------------------------1nd waypoint entered-----------------------------------------*/
                for (int j = 0; j <= 10; j++) {
                    cout << "Reached 1st Waypoint" << endl;
                }
                use_vision_msg.data = false;
                for(int j=0; j<100; j++){
                    use_vision_publisher.publish(use_vision_msg);
                }
                //change teb parameters
                system("bash teb_params_no_mans_land.sh");

                for (int j = 0; j <= 10; j++) {
                    cout << "Going towards 2nd waypoint" << endl;
                }
                spinOnce();

                
                /*-------------------------------------2nd waypoint-----------------------------------------*/
                radius2 = distance(current_lat, current_long, mid1_lat, mid1_long);
                spinOnce();
                int count2 = 0;
                while ((radius2 > radius && count2 <= 10)&& ros::ok()) {
                    for (int j = 0; j <= 10; j++) {
                        cout << "Still searching for 2nd waypoint, radius2: " << radius2 << endl;
                    }
                    gps_status = gps_waypoint(mid1_lat, mid1_long, current_lat, current_long, two_orient); 
                    //makes bot reach goal. Will retrun 0 if successful or 1 if not. Wont return until something happens
                    radius2 = distance(current_lat, current_long, mid1_lat, mid1_long);
                    if (radius2 < radius) {
                        count2++;
                    }
                    else {
                        count2 = 0;
                    }
                    spinOnce();
                }

                for (int j = 0; j <= 10; j++) {
                    cout << "Switching to vision after 2nd wpt" << endl;
                }
                system("bash teb_params_vision_middle.sh");
                use_vision_msg.data = true;
                for(int j=0; j<100; j++) {
                    use_vision_publisher.publish(use_vision_msg);
                }
            }
            
            /*---------------------------------------NO Mans Land Part 2-----------------------------------*/
            //entering no man's land 2nd part
            if (!flagstart2 && count2 >= 10)
            {
                int gps_status;
                flagstart2 = true;
                /*-------------------------------------3rd waypoint entered-----------------------------------------*/
                for (int j = 0; j <= 10; j++) {
                    cout << "Reached 3rd Waypoint" << endl;
                }
                use_vision_msg.data = false;
                for(int j=0; j<100; j++){
                    use_vision_publisher.publish(use_vision_msg);
                }
                //change teb parameters
                system("bash teb_params_no_mans_land.sh");

                for (int j = 0; j <= 10; j++) {
                    cout << "Going towards 4nd waypoint" << endl;
                }
                spinOnce();
                
                /*-------------------------------------4th waypoint-----------------------------------------*/
                radius4 = distance(current_lat, current_long, end_lat, end_long);
                spinOnce();

                int count4 = 0;
                while ((radius4 > radius && count4 <= 10) && ros::ok()) {
                    for (int j = 0; j <= 10; j++) {
                        cout << "Still searching for 4th waypoint, radius4: " << radius4 << endl;
                    }
                    gps_status = gps_waypoint(end_lat, end_long, current_lat, current_long, four_orient); 
                    radius4 = distance(current_lat, current_long, end_lat, end_long);
                    if (radius4 < radius) {
                        count4++;
                    }
                    else {
                        count4 = 0;
                    }
                    spinOnce();
                }

                for (int j = 0; j <= 10; j++) {
                    cout << "Switching to vision after 4th wpt" << endl;
                }
                use_vision_msg.data = true;
                system("bash teb_params_vision.sh");
                for(int j=0; j<100; j++) {
                    use_vision_publisher.publish(use_vision_msg);
                }
            }
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
