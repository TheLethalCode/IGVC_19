#ifndef HOUGH
#define HOUGH

#include"opencv2/highgui/highgui.hpp"
#include"opencv2/imgproc/imgproc.hpp"
#include"opencv2/core/core.hpp"
#include<iostream>
#include <opencv2/opencv.hpp>
#include <math.h>

using namespace cv;

/*

This is used when the vision module encouters a horizontal
  lane(since due to fixed x-axis, RANSAC can't fit them).
  
This module:
  * check_whether_hough: first draws the horizontal lane.
  * waypoint_for_hough: Generates a waypoint for the lane.

*/

double theta;

/*
The slope is -ve of what it is by convention.

The -ve sign is introduced to change the origin from 
  the image frame(top-left) to the co-ordinate 
  frame(bottom-left).
*/
double Slope(int x0, int y0, int x1, int y1)
{
  //NOTE the angle.
  return -1.0*(double)(y0-y1)/(x1-x0);

}

void fullLine(Mat img, Point a, Point b, Scalar color)
{
    double slope = Slope(a.x, a.y, b.x, b.y);

    Point p(0,0), q(img.cols, img.rows); // '->' used since pointer to image is used

    //(y1 - y0)= m(x1 - x0)
    p.y = (p.x - a.x) * slope + a.y;
    q.y = (q.x - b.x) * slope + b.y;
    //line(*img,p,q,color,3,CV_AA);

    //Setting slope range for which hough code will initiate
    line(img,p,q,color,3,CV_AA);
}

/*
Checks whether hough line is to be applied or not.
Takes for input:
    * Final output image of pre-processing (img)
    * Blank image for plotting the hough line (hough_img)
*/
Vec4i l;
bool check_whether_hough(Mat hough_img, Mat img, Parabola &lanes)
{
  vector<Vec4i> lines;  //NOTE the vector type
  
  //Hough Lines Probabilistic
  //cout << "starting houghlinesp" << endl;
  HoughLinesP(img, lines, 1, CV_PI/180, hough_min_points, hough_min_line_length, hough_max_line_gap); 
  //cout << "ending houghlinesp" << endl;

  /*
  Params: dest_img, vector to store pts, resolution of r,
         resolution of theta, minimum no. points, 
         min. line length, maxlineGap 
  */ 

  if(lines.size() == 0)
  {
    // cout << "----------------------\nNo hough line could be fit\n----------------------" << endl;
    return false;
  }

  Point p1(0,0), p2(0,0);
  double slope_avg=0;

  //Finding maximum line length & its index in lines[]
  //  We assume that the longest line will be the lane
  float max_dist = 0;
  int max_index = -1;
  for(size_t i = 0; i < lines.size(); i++)
  {
    Vec4i l = lines[i];
    //line(hough_img,Point(lines[i][0], lines[i][1]),Point(lines[i][2], lines[i][3]),Scalar(255),3,CV_AA);

    if(sqrt(pow(l[0]-l[2],2) + pow(l[1]-l[3],2)) > max_dist)
    {
      max_dist = sqrt( pow(l[0]-l[2],2) + pow(l[1]-l[3],2));
      max_index = i;
    }
  }
  //return false;

  if(lines.size() == 0 || max_index == -1)
  {
    // cout << "----------------------\nNo hough line could be fit\n----------------------" << endl;
    return false;
  }

  //Largest line= lane
  l = lines[max_index];
  p1.x = l[0];
  p1.y = l[1];
  p2.x = l[2];
  p2.y = l[3];

  double slope = (double)(p2.y*1.0 - p1.y*1.0)/(p2.x-p1.x + 1e-3);
  double angle = (atan(slope) * 180)/CV_PI;
  
  //cout << "reached till after angle: " << angle << endl;
  //Checking that the line slope is within a certain threshold angle from the x-axis
  if(angle <= 40.00 && angle >= -40.00) {

    if (angle < -10) {
      //cout << "khud se l" << endl;
      side = 'l';
      lanes.numModel = 1;
      lanes.a2=0;
      lanes.c2=0;
      lanes.a1=1;
      lanes.c1=1;

    }
    if (angle > 10) {
      //cout << "khud se r" << endl;
      side = 'r';
      lanes.numModel = 1;
      lanes.a1=0;
      lanes.c1=0;
      lanes.c2=1;
      lanes.a2=1;

    }
    return true;
  }
  return false; 
}

//Giving waypoint if hough initiated
NavPoint waypoint_for_hough(Mat img, char c, float theta, Parabola &lanes)
{

  Point p1, p2;
  p1.x = l[0];
  p1.y = l[1];
  p2.x = l[2];
  p2.y = l[3];

  double slope1 = (double)(p2.y*1.0 - p1.y*1.0)/(p2.x-p1.x + 1e-3);
  double angle1 = (atan(slope1) * 180)/CV_PI;

  //cout << "angle1: " << angle1 << endl;
  //cout << "angle: " << angle << endl;

  //Setting waypoint position + orientation
  NavPoint waypoint;

  if (c == 'l') {
    int upar = p1.y < p2.y ? p1.y:p2.y;
    waypoint.y = (img.rows + upar)/2;
    // cout << "hough_ratio_from_left: " << hough_ratio_from_left << endl;
    waypoint.x = hough_ratio_from_left * img.cols;

    // cout << "line nahi bani 1" << endl;
    // line(fitLanes,p1,p2,Scalar(255, 0, 0),3,CV_AA);

    fullLine(fitLanes, p1, p2, Scalar(255, 0, 0));
    if (costmap_publish_ransac) {
      fullLine(img, p1, p2, Scalar(255));
    }

    side = 'l';
    lanes.numModel = 1;
    lanes.a2=0;
    lanes.c2=0;
    lanes.a1=1;
    lanes.c1=1;

    // cout << "line ban gayi 1" << endl;

    // waypoint.y = upar;
    // if (upar == p1.y)
    //   waypoint.x = p1.x;
    // else 
    //   waypoint.x = p2.x;


  }
  else {
    int upar = p1.y < p2.y ? p1.y:p2.y;
    waypoint.y = (img.rows + upar)/2;
    // cout << "hough_ratio_from_left: " << hough_ratio_from_left << endl;
    waypoint.x = (1-hough_ratio_from_left) * img.cols;

    //cout << "line nahi bani 2" << endl;
    // line(fitLanes,p1, p2,Scalar(0, 0, 255),3,CV_AA);
    fullLine(fitLanes, p1, p2, Scalar(0, 0, 255));
    if (costmap_publish_ransac) {
      fullLine(img, p1, p2, Scalar(255));
    }

    side = 'r';
    lanes.numModel = 1;
    lanes.a1=0;
    lanes.c1=0;
    lanes.a2=1;
    lanes.c2=1;

    //cout << "line ban gayi 2" << endl;

    // waypoint.y = upar;
    // if (upar == p1.y)
    //   waypoint.x = p1.x;
    // else 
    //   waypoint.x = p2.x;
  }

  Mat transform(img.rows, img.cols, CV_8UC1, Scalar(0));

  circle(transform, p1, 3, Scalar(100), -1);
  circle(transform, p2, 3, Scalar(200), -1);

  Point p1_top, p2_top;
  transform = top_view(transform);
  for (int i = 0; i < img.rows; i++) {
    for (int j = 0; j < img.cols; j++) {
      if (transform.at<uchar>(i,j) == 100) {
        p1_top.x = j;
        p1_top.y = i;

      }
      if (transform.at<uchar>(i,j) == 200) {
        p2_top.x = j;
        p2_top.y = i;
      }
    }
  }

  // arrowedLine(costmap, origin, dest, Scalar(0,255,0), 3, 8, 0, 0.1);
  
  if (side == 'l') {
    line(frame_topview, Point(p1_top.x, p1_top.y), Point(p2_top.x, p2_top.y), Scalar(255, 0, 0) ,3, CV_AA);
  }
  else if (side == 'r') {
    line(frame_topview, Point(p1_top.x, p1_top.y), Point(p2_top.x, p2_top.y), Scalar(0, 0, 255) ,3, CV_AA);
  }
  // Point
  // arrowedLine(transform, Point(waypoint.x, waypoint.y), Point(p2_top.x, p2_top.y), Scalar(127), 3, 8, 0);
  // imshow("transformHoughed", transform);

  // Mat waypt1 = (Mat_<double>(3,1) << p1.x , p1.y , 1);
  // Mat waypt2 = (Mat_<double>(3,1) << p2.x , p2.y , 1);
  // Mat waypt_top1 = h*waypt1;
  // Mat waypt_top2 = h*waypt2;   

  // double slope = (double)(waypt_top2.at<double>(1,0)*1.0 - waypt_top1.at<double>(1,0)*1.0)/(waypt_top2.at<double>(0,0)*1.0 - waypt_top1.at<double>(0,0)*1.0);
  double slope = (double)(p2_top.y*1.0 - p1_top.y)/(p2_top.x - p1_top.x + 1e-3);
  double angle = atan(slope);
  
  //cout << "angle: " << angle*180/CV_PI << endl;



  // printf("1:%lf %lf 2:%lf %lf\n", waypt_top1.at<double>(0,0),  waypt_top1.at<double>(1,0),  waypt_top2.at<double>(0,0),  waypt_top2.at<double>(1,0));
  // line(img, )
  angle = -1 * angle;

  if (angle < 0) {
    angle += CV_PI;
  }

  angle -= CV_PI/2;

  // cout << "angle: " << (angle*180)/CV_PI << endl;


  // angle *= -1;
    
  if(side=='l') {
    angle = -(fabs(angle));
  }
  if(side=='r') {
    angle = (fabs(angle));
  }
  
  waypoint.angle = angle;
  // waypoint= h*waypoint;
  return waypoint;
  

}

#endif
