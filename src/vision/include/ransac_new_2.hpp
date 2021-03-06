#ifndef RANSAC_NEW_2
#define RANSAC_NEW_2


#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <bits/stdc++.h>
#include <tf/transform_datatypes.h>
#include "tf/transform_listener.h"

using namespace std;
using namespace cv;
Point centroid_inliers_r(0,0);
Point centroid_inliers_l(0,0);

#define dist_between_lanes 3


/*
 * This RANSAC model fits both lane curves simultaneously by taking 4 random points (2 on each lanes).
 * The parabolas are fit assuming bottom-left as origin: i.e. x towards right and y downwards.
 * The equation used for parabola is (y^2)= a(x-c).
 * Both a & c are found by solving simultaneous linear equations.
 */



int debug=0;
int change= 100;    //for sudden change function
double imu_orient_prev=0;

bool return_prev_params=0;
//structure to define the Parabola parameters
typedef struct Parabola
{
    int numModel = 0;   //no. of lanes
    float a1 = 0.0;
    float c1 = 0.0;
    float a2 = 0.0;
    // float b2 = 0.0;
    float c2 = 0.0;
}Parabola;

Parabola swap(Parabola &param) {

    float temp1, temp3;
    temp1=param.a1;
    temp3=param.c1;

    param.a1=param.a2;
    param.c1=param.c2;

    param.a2=temp1;
    param.c2=temp3;

    return param;
}

void printParabola(Parabola p) {
    // cout << "numModel: " << p.numModel << endl;
    // cout << "a1: " << p.a1 << "\tc1: " << p.c1 << endl;
    // cout << "a2: " << p.a2 << "\tc2: " << p.c2 << endl;
}


// previous.numModel=0;

/*
Parabola classify_lanes(Mat img,Parabola present,Parabola previous)
{
    float a1=present.a1;
    float a2=present.a2;
    float c1=present.c1;
    float c2=present.c2;
    int number_of_lanes=present.numModel;

    if(number_of_lanes==2)
    {
        if(c2<c1)
        {
            present=swap(present);
            return present;
        }
        else 
            return present;
    }

    else if(number_of_lanes==1)
    {
        //if intersection on left or right lane possible
        if( (a1*c1)<0 && ((a1*(img.cols-c1))>0) )
        {
            float y1=sqrt(-1.0*a1*c1);
            float y2=sqrt(a1*(img.cols-c1));

            // if(y1>(2*img.rows)/5 && y1<(3*img.rows)/5 && y2>(2*img.rows)/5 && y2<(3*img.rows)/5)
            if(y1>(2*img.rows)/5 && y2>(2*img.rows)/5)
            {
                return previous;
            }

        }

        if( (a2*c2)<0 && ((a2*(img.cols-c2))>0) )
        {

            float y1=sqrt(-1.0*a2*c2);
            float y2=sqrt(a2*(img.cols-c2));

            // if(y1>(2*img.rows)/5 && y1<(3*img.rows)/5 && y2>(2*img.rows)/5 && y2<(3*img.rows)/5)
            if(y1>(2*img.rows)/5 && y2>(2*img.rows)/5)
            {
                return previous;
            }
        }

        if((c1>(2*img.cols/5) && c1<(3*img.cols/5)) || (c2>(2*img.cols/5) && c2<(3*img.cols/5)))
        {
            return previous;
        }

        if(a1!=0 && c1>(img.cols/2))
        {
            present=swap(present);
            return present;
        }

        if(a2!=0 && c2<(img.cols/2))
        {
            present=swap(present);
            return present;
        }

    }

    return present;

}
*/


Parabola classify_lanes(Mat img,Parabola &present,Parabola &previous)
{

    /*
    cout << "-----------\nPrevious: " << endl;
    printParabola(previous);
    */

    float a1=present.a1;
    float a2=present.a2;
    float c1=present.c1;
    float c2=present.c2;
    int number_of_lanes=present.numModel;

    if (number_of_lanes == 1) {
        is_current_single = true;
    }

    if(number_of_lanes==2)
    {
        if(c2<c1)
        {
            present=swap(present);
            return present;
        }
        else 
            return present;
    }

    //if single lane is detected 
    else if(number_of_lanes==1)
    {
        imu_orient_prev = imu_orientation;
        //if a1 lane is detected and cuts to right, then make it right lane
        if(a1!=0 && c1!=0 && (c1>(0.85 * img.cols)))
        {
            is_current_single_left = false;
            cout << "cutting to right, making it right" << endl;
            present=swap(present);
            printParabola(present);
            return present;
        }

        else if (a1 != 0 && c1 != 0 && (c1 < 0.15*img.cols)) {
            return present;
        }

        //if a2 lane is detected and cuts to left, then make it left lane
        if (a2!=0 && c2!=0 && (c2<(0.15 * img.cols)))
        {
            is_current_single_left = true;
            cout << "cutting to left, making it left" << endl;
            present=swap(present);
            printParabola(present);
            return present;
        }

        else if (a2 != 0 && c2 != 0 && (c2 > 0.85*img.cols)) {
            return present;
        }

        //if the current single lane is ambiguous, and previous frame had one lane
        else if (previous.numModel == 1) {

            //if the previous frame had right single lane, and current lane is a1 then make it right
            if (a2 == 0 && c2 == 0 && previous.a1 == 0 && previous.c1 == 0) {
                is_current_single_left = false;
                cout << "prev frame 1 lane, current frame detected right" << endl;
                present=swap(present);
                printParabola(present);
                return present;
            }

            //if the previous frame had left single lane, and current lane is a2 then make it left
            else if (a1 == 0 && c1 == 0 && previous.a2 == 0 && previous.c2 == 0) {
                is_current_single_left = true;
                cout << "prev frame 1 lane, current frame detected left" << endl;
                present=swap(present);
                printParabola(present);
                return present;
            }

            if (a1 == 0 && previous.a1 == 0) {
                is_current_single_left = false;
            }
            else {
                is_current_single_left = true;
            }
            return present;

        }

        //previous had 2 lanes
        else {

            cout << "previous had 2 lanes" << endl;
            int current_bottom;
            if (a1 != 0) {
                current_bottom = c1;
            }
            else {
                current_bottom = c2;
            }

            int prev_left = previous.c1;
            int prev_right = previous.c2;

            //current single lane is closer to prev left
            if (abs(current_bottom - prev_left) < abs(current_bottom-prev_right)) {

                //current is right, make it left
                if (a2 != 0 && c2 != 0) {
                    is_current_single_left = true;
                    cout << "bottom nearer to right lane" << endl;
                    present=swap(present);
                    printParabola(present);
                }

                return present;
            }

            //current single lane is closer to prev right
            else {
                //current is left, make it right
                if (a1 != 0 && c1 != 0) {
                    is_current_single_left = false;
                    cout << "bottom nearer to right lane" << endl;
                    present=swap(present);
                    printParabola(present);
                }

                return present;

            }
        }

    }

    else {
        //Unbiased Classification in the case of no previous lane and 1 current lane
        if(a1!=0 && c1!=0 && (c1>(0.5 * img.cols)))
        {
            is_current_single_left = false;
            cout << "cutting to right, making it right" << endl;
            present=swap(present);
            printParabola(present);
            return present;
        }

        else if (a1 != 0 && c1 != 0 && (c1 < 0.5*img.cols)) {
            return present;
        }

        //if a2 lane is detected and cuts to left, then make it left lane
        if (a2!=0 && c2!=0 && (c2<(0.5 * img.cols)))
        {
            is_current_single_left = true;
            cout << "cutting to left, making it left" << endl;
            present=swap(present);
            printParabola(present);
            return present;
        }

        else if (a2 != 0 && c2 != 0 && (c2 > 0.5*img.cols)) {
            return present;
        }
    }


}


vector<Parabola> v;
// Parabola operator+(const Parabola &x)
// {
//     Parabola ans;
//     ans.numModel = numModel+x.numModel;
//     ans.a1= a1- x.a1;
//     ans.c1= c1- x.c1;
//     ans.a2= a2- x.a2;
//     ans.c2= c2- x.c2;

// }

Point centroid(float a,float c,Mat img);

float dist(Point A,Point B)
{
    return ((double)sqrt(pow(A.x-B.x,2)+pow(A.y-B.y,2)));
}

geometry_msgs::PointStamped ros_centroid_r; 
geometry_msgs::PointStamped ros_centroid_l; 
geometry_msgs::PointStamped ros_centroid_bl_r;
geometry_msgs::PointStamped ros_centroid_bl_l;
geometry_msgs::PointStamped ros_centroid_prev_r;
geometry_msgs::PointStamped ros_centroid_prev_l;

Parabola swap_odom(Parabola param) {

    float temp1, temp3;
    geometry_msgs::PointStamped temp; 
    Point temp_tt;
    temp1=param.a1;
    // temp2=param.b1;
    temp3=param.c1;

    param.a1=param.a2;
    // param.b1=param.b2;
    param.c1=param.c2;

    param.a2=temp1;
    // param.b2=temp2;
    param.c2=temp3;

    temp = ros_centroid_l;
    ros_centroid_l = ros_centroid_r;
    ros_centroid_r = temp;

    temp = ros_centroid_bl_l;
    ros_centroid_bl_l = ros_centroid_bl_r;
    ros_centroid_bl_r = temp;

    temp_tt = centroid_inliers_l;
    centroid_inliers_l = centroid_inliers_r;
    centroid_inliers_r = temp_tt;

    return param;
}

/*
   Calculate distance of passed point from curve
   Gives the minimum of the abs diff. in X/Y.
 */
float get_del(Point p, float a, float c)
{
    float predictedX = ((p.y*p.y)/(a) + c);
    float errorx = fabs(p.x - predictedX);

    float predictedY = sqrt(fabs(a*(p.x-c)));
    float errory = fabs(p.y - predictedY);

    return min(errorx, errory);
}

void stamped_point_make(Mat img, geometry_msgs::PointStamped& left, geometry_msgs::PointStamped& right, Parabola lanes ,vector<Point> ptArray1)
{
    int count_l=0;
    int count_r=0;
    centroid_inliers_l.x = 0;
    centroid_inliers_l.y = 0;
    centroid_inliers_r.x = 0;
    centroid_inliers_r.y = 0;
    cout<<"0" <<endl;


    //get current centroids(classified)
    if(lanes.numModel == 1)
    {
        cout<<"1" <<endl;
        if(lanes.a1 == 0  && lanes.c1 == 0)
        { 
            for(int i=0; i<ptArray1.size(); i++)
            {
                if(get_del(ptArray1[i],lanes.a2,lanes.c2)<maxDist)
                {
                    Mat waypt = (Mat_<double>(3,1) << ptArray1[i].x , img.rows - ptArray1[i].y , 1);
                    Mat waypt_top = h*waypt;

                    double x_top = waypt_top.at<double>(0,0)/waypt_top.at<double>(2,0);
                    double y_top = waypt_top.at<double>(1,0)/waypt_top.at<double>(2,0);
                    Point top(x_top, y_top);                                
                    centroid_inliers_r.x += top.x;
                    centroid_inliers_r.y += top.y;
                    count_r++;
                }
            }
            centroid_inliers_r.x = centroid_inliers_r.x/count_r;
            centroid_inliers_r.y = centroid_inliers_r.y/count_r;
            ros_centroid_bl_r.point.x = (img.rows - centroid_inliers_r.y)/pixelsPerMetre;
            ros_centroid_bl_r.point.y = (img.cols/2 - centroid_inliers_r.x)/pixelsPerMetre;
            ros_centroid_bl_l.point.x = 0;
            ros_centroid_bl_l.point.y = 0;
        }   

        else
        {
            for(int i=0; i<ptArray1.size(); i++)
            {
                if(get_del(ptArray1[i],lanes.a1,lanes.c1)<maxDist)
                {
                    Mat waypt = (Mat_<double>(3,1) << ptArray1[i].x , img.rows - ptArray1[i].y , 1);
                    Mat waypt_top = h*waypt;

                    double x_top = waypt_top.at<double>(0,0)/waypt_top.at<double>(2,0);
                    double y_top = waypt_top.at<double>(1,0)/waypt_top.at<double>(2,0);
                    Point top(x_top, y_top);
                    centroid_inliers_l.x += top.x;
                    centroid_inliers_l.y += top.y;
                    count_l++;
                }
            }
            centroid_inliers_l.x = centroid_inliers_l.x/count_l;
            centroid_inliers_l.y = centroid_inliers_l.y/count_l; 
            ros_centroid_bl_l.point.x = (img.rows - centroid_inliers_l.y)/pixelsPerMetre;
            ros_centroid_bl_l.point.y = (img.cols/2 - centroid_inliers_l.x)/pixelsPerMetre;
            ros_centroid_bl_r.point.x = 0;
            ros_centroid_bl_r.point.y = 0;
        }
    }

    if(lanes.numModel == 2)
    {
        cout<<"2" <<endl;
        for(int i=0; i<ptArray1.size(); i++)
        {
            if(get_del(ptArray1[i],lanes.a2,lanes.c2)<maxDist)
            {
                Mat waypt = (Mat_<double>(3,1) << ptArray1[i].x , img.rows - ptArray1[i].y , 1);
                Mat waypt_top = h*waypt;

                double x_top = waypt_top.at<double>(0,0)/waypt_top.at<double>(2,0);
                double y_top = waypt_top.at<double>(1,0)/waypt_top.at<double>(2,0);
                Point top(x_top, y_top);                                
                centroid_inliers_r.x += top.x;
                centroid_inliers_r.y += top.y;
                count_r++;
            }
        }
        centroid_inliers_r.x = centroid_inliers_r.x/count_r;
        centroid_inliers_r.y = centroid_inliers_r.y/count_r;
        ros_centroid_bl_r.point.x = (img.rows - centroid_inliers_r.y)/pixelsPerMetre;
        ros_centroid_bl_r.point.y = (img.cols/2 - centroid_inliers_r.x)/pixelsPerMetre;

        for(int i=0; i<ptArray1.size(); i++)
        {
            if(get_del(ptArray1[i],lanes.a1,lanes.c1)<maxDist)
            {
                Mat waypt = (Mat_<double>(3,1) << ptArray1[i].x , img.rows - ptArray1[i].y , 1);
                Mat waypt_top = h*waypt;

                double x_top = waypt_top.at<double>(0,0)/waypt_top.at<double>(2,0);
                double y_top = waypt_top.at<double>(1,0)/waypt_top.at<double>(2,0);
                Point top(x_top, y_top);
                centroid_inliers_l.x += top.x;
                centroid_inliers_l.y += top.y;
                count_l++;
            }
        }
        centroid_inliers_l.x = centroid_inliers_l.x/count_l;
        centroid_inliers_l.y = centroid_inliers_l.y/count_l;
        ros_centroid_bl_l.point.x = (img.rows - centroid_inliers_l.y)/pixelsPerMetre;
        ros_centroid_bl_l.point.y = (img.cols/2 - centroid_inliers_l.x)/pixelsPerMetre;

    }
}
Parabola classify_lanes_odom(Mat img,Parabola present,Parabola previous, vector<Point> ptArray1)
{
    Point a,b,c;
    ros_centroid_bl_l.header.frame_id = "/base_link";
    ros_centroid_bl_r.header.frame_id = "/base_link";
    ros_centroid_bl_l.header.stamp = ros::Time();
    ros_centroid_bl_r.header.stamp = ros::Time();

    float a1=present.a1;
    float a2=present.a2;
    float c1=present.c1;
    float c2=present.c2;

    present=classify_lanes(img, present, previous);
    stamped_point_make(img, ros_centroid_bl_l, ros_centroid_bl_r, present, ptArray1);
    cout << "Current Centroids in base_link frame\n" << endl;
    cout<< "ros_centroid_bl_l: " << ros_centroid_bl_l.point.x << " " <<  ros_centroid_bl_l.point.y << endl;
    cout<< "ros_centroid_bl_r: " << ros_centroid_bl_r.point.x << " " <<  ros_centroid_bl_r.point.y << endl;

    //convert centroids to odom frame and store
    static tf::TransformListener listener;
    try
    {
        listener.transformPoint("/odom", ros_centroid_bl_l, ros_centroid_l);
        listener.transformPoint("/odom", ros_centroid_bl_r, ros_centroid_r);

        // ROS_INFO("point_base: (%.2f, %.2f) -----> point_odom: (%.2f, %.2f)",ros_centroid_bl_l.point.x, ros_centroid_bl_l.point.y,ros_centroid_l.point.x, ros_centroid_l.point.y);
    }
    catch(tf::TransformException& ex)
    {
        ROS_ERROR("%s", ex.what());
    }

    //markers publishing current centroids
    geometry_msgs::Point p1, p2;
    p1.x = ros_centroid_l.point.x;   
    p1.y = ros_centroid_l.point.y;       
    p1.z = 0; 
    p2.x = ros_centroid_r.point.x;   
    p2.y = ros_centroid_r.point.y;       
    p2.z = 0; 
    points1.points.clear();
    points2.points.clear();

    points1.points.push_back(p1);
    points2.points.push_back(p2);
    marker_pub1.publish(points1);
    marker_pub2.publish(points2);

    // if its the first iteration make previous equal to current to avoid any problems
    //return the output from classify_lanes in the first iteration
    static int count = 0;
    if(count < 10)
    {
        ros_centroid_prev_r.point.x = ros_centroid_r.point.x;
        ros_centroid_prev_r.point.y = ros_centroid_r.point.y;
        ros_centroid_prev_l.point.x = ros_centroid_l.point.y;
        ros_centroid_prev_l.point.y = ros_centroid_l.point.y; 
        count ++;
        return present;
    }

    
    cout << "<<<<<<<<<<<<<<<<<<<<<<<Checking Distance of Previous centroids from current>>>>>>>>>>>>>>>>>>>>>>>>>>>>" << endl;
    Point pl, pr, l, r;
    pl.x=ros_centroid_prev_l.point.x;
    pl.y=ros_centroid_prev_l.point.y;

    pr.x=ros_centroid_prev_r.point.x;
    pr.y=ros_centroid_prev_r.point.y;

    l.x=ros_centroid_l.point.x;
    l.y=ros_centroid_l.point.y;

    r.x=ros_centroid_r.point.x;
    r.y=ros_centroid_r.point.y;
    cout << "previous distances: " << dist(pl, l) << " " << dist(pr, r) << endl;
    // return present;

    if(present.numModel==1)
    {
        if(previous.numModel == 0) //old methods
        {
            //check if previous frame was updated within 3-4 frames back, if yes then we can apply the distance like
            //calculate distance between current centroid and previous stored centroid, if its less than 3m then NO_SWAP
            //if the distance is greater than 3m then swap the lanes 
        }

        if(previous.numModel == 1)
        {
            cout << "1111111111----------11111111111" << endl;
            if(present.a1 == 0 && present.c1 == 0) //current lane detected is right
            {
                //store current right lane centroids in a
                a.x = ros_centroid_r.point.x; 
                a.y = ros_centroid_r.point.y; 

                if(previous.a1 == 0 && previous.c1 == 0) //previous lane was right
                {
                    //store previous right lanes params in b
                    b.x = ros_centroid_prev_r.point.x;
                    b.y = ros_centroid_prev_r.point.y;
                    cout<<"dist(a,b)  : "<<dist(a,b)<<endl;
                    if(dist(a,b) < dist_between_lanes)
                        return present;
                    else
                    {
                        present = swap_odom(present);
                        return present;
                    } 

                }
                else //previous lane was left
                {
                    //store previous left params in b
                    b.x = ros_centroid_prev_l.point.x;
                    b.y = ros_centroid_prev_l.point.y;
                    cout<<"dist(a,b)  : "<<dist(a,b)<<endl;
                    if(dist(a,b) < dist_between_lanes)
                    {
                        present = swap_odom(present);
                        return present;
                    }
                    else
                        return present;
                }
            }
            else if(present.a2 == 0 && present.c2 == 0) //current lane is left according to us
            {
                //store current left 
                a.x = ros_centroid_l.point.x;
                a.y = ros_centroid_l.point.y;
                if(previous.a2 == 0 && previous.c2 == 0)
                {
                    b.x = ros_centroid_prev_l.point.x;
                    b.y = ros_centroid_prev_l.point.y;
                    cout<<"dist(a,b)  : "<<dist(a,b)<<endl;
                    if(dist(a,b) < dist_between_lanes)
                        return present;
                    else
                    {
                        present = swap_odom(present);
                        return present;
                    }
                }
                else
                {
                    b.x = ros_centroid_prev_r.point.x;
                    b.y = ros_centroid_prev_r.point.y;
                    cout<<"dist(a,b)  : "<<dist(a,b)<<endl;
                    if(dist(a,b) < dist_between_lanes)
                    {
                        present = swap_odom(present);
                        return present;
                    }
                    else
                        return present;
                }
            }
        }
    }
    // cout<<"ros_centroid_bl_l.point.x, ros_centroid_bl_l.point.y, ros_centroid_bl_l.point.z : "<<ros_centroid_bl_l.point.x<<","<<ros_centroid_bl_l.point.y<<","<<ros_centroid_bl_l.point.z<<endl;
    // cout<<"ros_centroid_bl_r.point.x, ros_centroid_bl_r.point.y, ros_centroid_bl_r.point.z : "<<ros_centroid_bl_r.point.x<<","<<ros_centroid_bl_r.point.y<<","<<ros_centroid_bl_r.point.z<<endl;
    // cout<<"ros_centroid_r.point.x, ros_centroid_r.point.y, ros_centroid_r.point.z : "<<ros_centroid_r.point.x<<","<<ros_centroid_r.point.y<<","<<ros_centroid_r.point.z<<endl;
    // cout<<"ros_centroid_l.point.x, ros_centroid_l.point.y, ros_centroid_l.point.z : "<<ros_centroid_l.point.x<<","<<ros_centroid_l.point.y<<","<<ros_centroid_l.point.z<<endl;
    ros_centroid_prev_r = ros_centroid_r;
    ros_centroid_prev_l = ros_centroid_l;
    // namedWindow("noob",0);
    // waitKey(0);
    return present;
}

// Parabola classify_lanes_odom(Mat img,Parabola present,Parabola previous, vector<Point> ptArray1)
// {
//     old_
//     present=classify_lanes(img, present, previous);
//     //applying odom check for 1-1 and 0-1 case
//     //for that we need to store classified centroids of the previous frame and calculate centroid of the current frame

// }

//Getting a (purely mathematical)
float get_a(Point p1, Point p2)
{
    int x1 = p1.x;
    int x2 = p2.x;
    // int x3 = p3.x;
    int y1 = p1.y;
    int y2 = p2.y;
    // int y3 = p3.y;

    float del = (y1 - y2)*(y1 + y2);
    float del_a = (x1 - x2);
    float a;
    a = del/(del_a);

    if(fabs(a)>200000000)
        return FLT_MAX;
    else
        return a;
}

//Purely Mathematical
float get_c(Point p1, Point p2)
{
    int x1 = p1.x;
    int y1 = p1.y;

    int x2 = p2.x;
    int y2 = p2.y;

    float del = (x1 - x2)*y2*y2;
    float del_a = (y1 - y2)*(y1 + y2);

    return (x2 - (del/(del_a)));
}


//If the RANSAC is to be fit using only 3 points
//  assusming the curves to be parallel
float get_c_2(Point p1, float a)
{
    float c = p1.x - ((p1.y)*(p1.y))/a;

    return c;
}

float min(float a, float b){
    if(a<=b)
        return a;
    return b;
}


//removes both the lanes if they intersect within the image frame
bool isIntersectingLanes(Mat img, Parabola param) {
    float a1 = param.a1;
    float c1 = param.c1;

    float a2 = param.a2;
    float c2 = param.c2;

    if(a1==a2)
        return false;
    float x = (a1*c1 - a2*c2)/(a1-a2);

    //checks if intersection is within

    float y_2 = a1*(x-c1);

    if (y_2 > 0 &&  sqrt(y_2) < (img.rows) && x > 0 && x < img.cols) return true;
    return false;
}

bool isIntersectingLanes_2(Mat img, Parabola param){
    float a1 = param.a1;
    float c1 = param.c1;

    float a2 = param.a2;
    float c2 = param.c2;
    int x1,x2,y;

    for(int i=0; i<img.rows; i++){
        y = img.rows - i;
        x1 = ((y*y)/a1) + c1;
        x2 = ((y*y)/a2) + c2;
        if(fabs(x1 - x2) < 5)
        {
            //cout << fabs(x1-x2);
            return true;
        }
    }
    return false;
}

Parabola no_sudden_change(Parabola bestTempParam, Mat img, Parabola previous)
{
    if(bestTempParam.numModel==2 && previous.numModel == 2)
    {
        bestTempParam = classify_lanes(img, bestTempParam, previous);
        if(fabs(bestTempParam.a1 - previous.a1) > change || fabs(bestTempParam.c1 - previous.c1) > change)
        {
            bestTempParam.a1 = 0;
            bestTempParam.c1 = 0;
            bestTempParam.numModel--;
        }
        if(fabs(bestTempParam.a2 - previous.a2) > change || fabs(bestTempParam.c2 - previous.c2) > change)
        {
            bestTempParam.a2 = 0;
            bestTempParam.c2 = 0;
            bestTempParam.numModel--;
        }
    }

    else if(bestTempParam.numModel == 2 && previous.numModel == 1)
    {
        bestTempParam = classify_lanes(img, bestTempParam, previous);
        if((previous.a1 ==0 && previous.c1==0))
        {
            if(fabs(bestTempParam.a2 - previous.a2) > change || fabs(bestTempParam.c2 - previous.c2) > change)
            {
                bestTempParam.a2 = 0;
                bestTempParam.c2 = 0;
                bestTempParam.numModel--;
            }
        }
        else
        {
            if(fabs(bestTempParam.a1 - previous.a1) > change || fabs(bestTempParam.c1 - previous.c1) > change)
            {
                bestTempParam.a1 = 0;
                bestTempParam.c1 = 0;
                bestTempParam.numModel--;
            }
        }
    }

    else if(bestTempParam.numModel == 1 && previous.numModel == 2)
    {
        bestTempParam = classify_lanes(img, bestTempParam, previous);
        if(bestTempParam.a1 == 0 && bestTempParam.c1 == 0)
        {
            if(fabs(bestTempParam.a2 - previous.a2) > change || fabs(bestTempParam.c2 - previous.c2) > change)
            {
                bestTempParam.a2 = 0;
                bestTempParam.c2 = 0;
                bestTempParam.numModel--;
            }
        }
        else
        {
            if(fabs(bestTempParam.a1 - previous.a1) > change || fabs(bestTempParam.c1 - previous.c1) > change)
            {
                bestTempParam.a1 = 0;
                bestTempParam.c1 = 0;
                bestTempParam.numModel--;
            }
        }
    }

    else if(bestTempParam.numModel == 1 && previous.numModel == 1)
    {
        bestTempParam = classify_lanes(img, bestTempParam, previous);
        if((bestTempParam.a1 == 0 && bestTempParam.c1 == 0) && (previous.a1 == 0 && previous.c1 == 0))
        {
            if(fabs(bestTempParam.a2 - previous.a2) > change || fabs(bestTempParam.c2 - previous.c2) > change)
            {
                bestTempParam.a2 = 0;
                bestTempParam.c2 = 0;
                bestTempParam.numModel--;
            }
        }
        else if((bestTempParam.a2 == 0 && bestTempParam.c2 == 0) && (previous.a2 == 0 && previous.c2 == 0))
        {
            if(fabs(bestTempParam.a1 - previous.a1) > change || fabs(bestTempParam.c1 - previous.c1) > change)
            {
                bestTempParam.a1 = 0;
                bestTempParam.c1 = 0;
                bestTempParam.numModel--;
            }
        }
    }
    return bestTempParam;
}

//choose Parabola parameters of best fit curve basis on randomly selected 4 points
Parabola ransac(vector<Point> ptArray, Parabola &param, Mat img, Parabola &previous)
{
    int numDataPts = ptArray.size();

    Parabola bestTempParam;

    bestTempParam.numModel=2;

    int score_gl = 0;
    int score_l_gl = 0, score_r_gl = 0;

    // loop of iterations
    for(int i = 0; i < iteration; i++)
    {
        //Taking 4 random points
        int p1 = random()%ptArray.size(), p2 = random()%ptArray.size(), p3 = random()%ptArray.size(), p4 = random()%ptArray.size();

        // Checking if all the points are equal
        if(p1==p2 || p1==p3 || p1==p4 || p3==p2 || p4==p2 || p3==p4){
            i--;
            continue;
        }

        Point ran_points[4];
        ran_points[0] = ptArray[p1];
        ran_points[1] = ptArray[p2];
        ran_points[2] = ptArray[p3];
        ran_points[3] = ptArray[p4];

        int flag = 0;
        Point temp;

        //bubble sorting the points according to x co-ordinate
        for(int m = 0; m < 3; m++)
        {  
            for(int n = 0; n < 3 - m; n++)
            {
                if(ran_points[n].x > ran_points[n+1].x)
                {
                    temp = ran_points[n];
                    ran_points[n] = ran_points[n+1];
                    ran_points[n+1] = temp;
                }
            }  
        }

        /*
           Checking if points for the lane have the same x or y co-ordinates
         * Same x co-ordinates => infinite no. of parabolas.
         * Same y co-ordinates isn't possible due to a fixed axis(bottom row).
         */  
        if(ran_points[0].x == ran_points[1].x || ran_points[2].x==ran_points[3].x || ran_points[0].y == ran_points[1].y || ran_points[2].y==ran_points[3].y){
            i--;
            continue;
        }

        Parabola tempParam;
        tempParam.numModel = 2;
        tempParam.a1 = get_a(ran_points[0], ran_points[1]);
        tempParam.c1 = get_c(ran_points[0], ran_points[1]);


        //Just switch for true & false to plot lanes using 3 points(assuming parallel curves)
        if(true){
            tempParam.a2 = get_a(ran_points[2], ran_points[3]);
            tempParam.c2 = get_c(ran_points[2], ran_points[3]);
        }
        else if(false){
            tempParam.a2 = tempParam.a1;
            tempParam.c2 = get_c_2(ran_points[2], tempParam.a2);
        }


        int score_common = 0;
        int score_l_loc = 0, score_r_loc = 0;

        //looping over image
        for(int p = 0; p < ptArray.size(); p++)
        {
            int flag_l = 0; //for points on 1st curve
            int flag_r = 0; //for points on 2nd curve

            float dist_l = get_del(ptArray[p], tempParam.a1, tempParam.c1);

            if(dist_l < maxDist)
            {
                flag_l = 1;
            }

            float dist_r = get_del(ptArray[p], tempParam.a2, tempParam.c2);

            if(dist_r < maxDist)
            {
                flag_r = 1;
            }


            if(flag_l == 1 && flag_r == 1) 
            {
                score_common++;
            }

            //Common not added to l or r loc
            else 
            {
                if (flag_l == 1) {
                    score_l_loc++;
                }
                if (flag_r == 1) {
                    score_r_loc++;
                }
            }
        } //end of loop over image

        // // check if the centroids of the 2 parabolas are too close.
        // if (dist(centroid(tempParam.a1,tempParam.c1,img),centroid(tempParam.a2,tempParam.c2,img)) < 20.0){
        //     //cout<<"centroid issue."<<endl;
        //     if(score_r_loc > score_l_loc)
        //     {
        //         tempParam.a1 = 0;
        //         tempParam.c1 = 0;
        //         score_l_loc = 0;
        //         tempParam.numModel--;
        //     }
        //     else
        //     {
        //         tempParam.a2 = 0;
        //         tempParam.c2 = 0;
        //         score_r_loc = 0;
        //         tempParam.numModel--;
        //     }
        // }

        // Checking if the curves are too close at the bottom
        if (fabs(tempParam.c1 - tempParam.c2) < 40.0){
            // cout<<"c1-c2 issue. "<<fabs(tempParam.c1 - tempParam.c2)<<endl;
            if(score_r_loc > score_l_loc)
            {
                tempParam.a1 = 0;
                tempParam.c1 = 0;
                score_l_loc = 0;
                tempParam.numModel--;
            }

            else
            {
                tempParam.a2 = 0;
                tempParam.c2 = 0;
                score_r_loc = 0;
                tempParam.numModel--;
            }
        }

        //To check if intersection in image has taken place
        // Uncomment the block to add the intersection functionality.


        // else if( isIntersectingLanes_2(img, tempParam)) {
        //     //cout<<" isIntersectingLanes issue."<<endl;
        //     if(score_r_loc > score_l_loc)
        //     {
        //         tempParam.a1 = 0;
        //         tempParam.c1 = 0;
        //         score_l_loc = 0;
        //         tempParam.numModel--;
        //     }
        //     else
        //     {
        //         tempParam.a2 = 0;
        //         tempParam.c2 = 0;
        //         score_r_loc = 0;
        //         tempParam.numModel--;
        //     }
        // }

        //Max. Percentage Common Inliers Thresholding
        if ((score_common/(score_common + score_l_loc + score_r_loc+1))*100 > common_inliers_thresh) {
            // cout<<"common points issue."<<endl;
            if(score_r_loc > score_l_loc)
            {
                tempParam.a1 = 0;
                tempParam.c1 = 0;
                score_l_loc = 0;
                tempParam.numModel--;
            }
            else
            {
                tempParam.a2 = 0;
                tempParam.c2 = 0;
                score_r_loc = 0;
                tempParam.numModel--;
            }
        }

        /*else if(fabs(tempParam.a1 - tempParam.a2) < 200)
          {
        //cout<<"a1 - a2 issue."<<endl;
        if(score_r_loc > score_l_loc)
        {
        tempParam.a1 = 0;
        tempParam.c1 = 0;
        score_l_loc = 0;
        tempParam.numModel--;
        }
        else
        {
        tempParam.a2 = 0;
        tempParam.c2 = 0;
        score_r_loc = 0;
        tempParam.numModel--;
        }
        }*/


        // if(fabs(tempParam.a1) <120 && fabs(tempParam.c1) > 150 && score_l_loc!=0)
        // {
        //     cout<<"horizontal issue."<<endl;
        //     tempParam.a1 = 0;
        //     tempParam.c1 = 0;
        //     score_l_loc = 0;
        //     tempParam.numModel--;
        // }

        // if(fabs(tempParam.a2) <120 && fabs(tempParam.c2) > 150 && score_r_loc!=0)
        // {
        //     cout<<"horizontal issue."<<endl;
        //     tempParam.a2 = 0;
        //     tempParam.c2 = 0;
        //     score_r_loc = 0;
        //     tempParam.numModel--;
        // }

        // Checking params for bestTempParam
        if (tempParam.numModel==2 && (score_l_loc + score_r_loc ) > 2*score_gl) {

            score_l_gl=score_l_loc;
            score_r_gl=score_r_loc;
            score_gl = (score_r_gl + score_l_gl)/2;

            bestTempParam.a1=tempParam.a1;
            bestTempParam.c1=tempParam.c1;
            bestTempParam.a2=tempParam.a2;
            bestTempParam.c2=tempParam.c2;
            bestTempParam.numModel = tempParam.numModel;
        }
        if (tempParam.numModel==1 && (score_l_loc + score_r_loc) > score_gl) {

            score_l_gl=score_l_loc;
            score_r_gl=score_r_loc;
            score_gl = score_r_gl + score_l_gl;

            bestTempParam.a1=tempParam.a1;
            bestTempParam.c1=tempParam.c1;
            bestTempParam.a2=tempParam.a2;
            bestTempParam.c2=tempParam.c2;
            bestTempParam.numModel = tempParam.numModel;
        }
    } //end of iteration loop


    //Checking for imn. no. of inliers
    if(score_l_gl!=0 && (score_l_gl) < minLaneInlier){
        // cout<<"left lane removed"<< endl;
        bestTempParam.a1=0;
        bestTempParam.c1=0;
        bestTempParam.numModel--;
    }
    if(score_r_gl!=0 && (score_r_gl) < minLaneInlier){
        // cout<<"right lane removed"<<endl;
        bestTempParam.a2=0;
        bestTempParam.c2=0;
        bestTempParam.numModel--;
    }

    cout  <<"score_l_gl : "<< score_l_gl <<" score_r_gl : "<< score_r_gl <<endl;


    // if(true){
    //     cout<<"bestTempParam.numModel : "<<bestTempParam.numModel<<endl;
    //     cout<<"bestTempParam.a1 : "<<bestTempParam.a1<<" bestTempParam.c1 : "<<bestTempParam.c1<<endl;
    //     cout<<"bestTempParam.a2 : "<<bestTempParam.a2<<" bestTempParam.c2 : "<<bestTempParam.c2<<endl;
    // }
    return bestTempParam;
}

Point centroid(float a,float c,Mat img)
{
    Point A;
    int i,j,x,y;
    int sum_x = 0,sum_y = 0,count=1;

    for(j=0;j<img.rows;j++)
    {
        y = img.rows-j;
        x = ((y*y)/(a) + c);    

        if(x>=0 && x<img.cols)
        {
            sum_y+=y;
            sum_x+=x;
            count++;
        }
    }

    A.x=sum_x/count;
    A.y=sum_y/count;

    return A;
}



Parabola getRansacModel(Mat img,Parabola &previous, std::vector<Point> &ptArray1)
{
    //apply ransac for first time it will converge for one lane

    //Vector for storing white points
    ptArray1.clear();
    if(debug!=0)
    { 
        //cout << "---------------------------------NEW RANSAC----------------------------------" << endl;
    }
    // if (grid_white_thresh >= grid_size*grid_size) {
    //     grid_white_thresh = grid_size*grid_size -1;
    // }

    Mat plot_grid(img.rows,img.cols,CV_8UC1,Scalar(0));


    int count = 0;
    //Iterating through all the grid elements
    for(int i=((grid_size-1)/2);i<img.rows-(grid_size-1)/2;i+=grid_size)
    {
        for(int j=((grid_size-1)/2);j<img.cols-(grid_size-1)/2;j+=grid_size)
        {
            count=0;
            for(int x=(j-(grid_size-1)/2);x<=(j+(grid_size-1)/2);x++)
            {

                for(int y=(i-(grid_size-1)/2);y<=(i+(grid_size-1)/2);y++)
                {
                    if(img.at<uchar>(y,x)>wTh){
                        count++;
                    }
                    img.at<uchar>(y,x)=0;
                }
            }

            if(count>grid_white_thresh) {
                ptArray1.push_back(Point(j , img.rows - i));
                plot_grid.at<uchar>(i, j) = 255;

            }
        }
    }

    if (is_debug || is_important) {
        namedWindow("grid",WINDOW_NORMAL);
        imshow("grid",plot_grid);
    }


    //declare a Parabola variable to store the Parabola
    Parabola param;
    param.numModel = 0;
    param.a1 = 0;
    param.c1 = 0;
    param.a2 = 0;
    param.c2 = 0;

    //get parameters of first Parabola form ransac function

    //Checking if we have sufficient no. of points to initialise RANSAC
    if(ptArray1.size() > minPointsForRANSAC )
    {
        return_prev_params = 1;
        param = ransac(ptArray1, param, img, previous);
    }

    else {
        //return previous;
    }

    return param;
}

Mat drawLanes(Mat output, Parabola lanes) 
{

    vector<Point2f> left_lane, right_lane;
    float a1 = lanes.a1, a2 = lanes.a2, c1 = lanes.c1, c2 = lanes.c2;
    for (int j = 0; j < output.rows; j++){

        float x, y;
        if (a1 != 0 && c1 != 0) {
            y = output.rows - j;
            x = (y*y)/(a1) + c1;
            left_lane.push_back(Point2f(x, j));
        }

        if (a2 != 0 && c2 != 0) {
            y = output.rows - j;
            x = (y*y)/(a2) + c2;
            right_lane.push_back(Point2f(x, j));
        }

    }

    //Left lane in Blue
    Mat left_curve(left_lane, true);
    left_curve.convertTo(left_curve, CV_32S); //adapt type for polylines
    polylines(output, left_curve, false, Scalar(255, 0, 0), 3, CV_AA);

    //Right lane in Red
    Mat right_curve(right_lane, true);
    right_curve.convertTo(right_curve, CV_32S); //adapt type for polylines
    polylines(output, right_curve, false, Scalar(0, 0, 255), 3, CV_AA);

    return output;
}

Mat drawLanes_white(Mat img, Parabola lanes) {

    vector<Point2f> left_lane, right_lane;
    float a1 = lanes.a1, a2 = lanes.a2, c1 = lanes.c1, c2 = lanes.c2;

    for (int j = 0; j < img.rows; j++){

        float x, y;
        if (a1 != 0 && c1 != 0) {
            y = img.rows - j;
            x = (y*y)/(a1) + c1;
            left_lane.push_back(Point2f(x, j));
        }

        if (a2 != 0 && c2 != 0) {
            y = img.rows - j;
            x = (y*y)/(a2) + c2;
            right_lane.push_back(Point2f(x, j));
        }

    }

    Mat left_curve(left_lane, true);
    left_curve.convertTo(left_curve, CV_32S); //adapt type for polylines
    polylines(img, left_curve, false, Scalar(255), 3, CV_AA);

    Mat right_curve(right_lane, true);
    right_curve.convertTo(right_curve, CV_32S); //adapt type for polylines
    polylines(img, right_curve, false, Scalar(255), 3, CV_AA);

    return img;
}


#endif
