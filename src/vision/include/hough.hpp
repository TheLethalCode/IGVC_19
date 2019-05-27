#ifndef HOUGH
#define HOUGH

#include"opencv2/highgui/highgui.hpp"
#include"opencv2/imgproc/imgproc.hpp"
#include"opencv2/core/core.hpp"
#include<iostream>
#include <opencv2/opencv.hpp>
#include <math.h>

using namespace cv;
double theta;
double Slope(int x0, int y0, int x1, int y1)
{
     return (double)(y0-y1)/(x1-x0);
}

double fullLine(Mat *img, Point a, Point b, Scalar color)
{
     double slope = Slope(a.x, a.y, b.x, b.y);

     Point p(0,0), q(img->cols,img->rows);

     p.y = -(a.x - p.x) * slope + a.y;
     q.y = -(b.x - q.x) * slope + b.y;
     //line(*img,p,q,color,3,CV_AA);
     if((atan(slope)*180/CV_PI) <= 20.00 && (atan(slope)*180/CV_PI) >= -20.00)
     	line(*img,p,q,color,3,CV_AA);
     theta = atan(slope)*180/CV_PI;
     return slope;
}
bool check_whether_hough(Mat &img)
{
	//Canny(img, img, 50, 200, 3);
	vector<Vec4i> lines;
  	HoughLinesP(img, lines, 5, CV_PI/180, 50, 50, 10);
  	Point p1(0,0), p2(0,0);
  	double slope_avg=0;
 //  	Mat hough(img.rows,img.cols,CV_8UC1,Scalar(0));
 //  	for( size_t i = 0; i < lines.size(); i++ )
	// {
 //  		Vec4i l = lines[i];
 //  		line( hough, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(255), 3, CV_AA);
	// }
	// namedWindow("hough lines",0);
 // 	imshow("hough lines",hough);
    if(lines.size() == 0)
    {
        return false;
    }
 	Vec4i l = lines[0];
 	p1.x = l[0];
 	p1.y = l[1];
 	p2.x = l[2];
 	p2.y = l[3];
  	Mat img_hough(img.rows,img.cols,CV_8UC1,Scalar(0));
  	double slope = fullLine(&img_hough, p1, p2, Scalar(255));
  // 	namedWindow("img_hough",0);
 	// imshow("img_hough",img_hough);
 	// waitKey(10);
 	if((atan(slope)*180/CV_PI) <= 20.00 && (atan(slope)*180/CV_PI) >= -20.00)
 	{
 		img = img_hough.clone();
 		return true;
 	}
  		
}

NavPoint waypoint_for_hough(Mat img, char c, float theta)
{
    int ver_i,ver_j;
    for(int i=img.rows -1;i>=0;i--)
    {
        if(img.at<uchar>(i,img.cols/2) == 255)
        {
            ver_i = i;
            ver_j = img.cols/2;
            break;
        }
    }
    int waypoint_i,waypoint_j;
    // cout<<c<<endl;
    if(c == 'l')
    {
        waypoint_i = (ver_i + 2*img.rows -1)/3;
        waypoint_j = (ver_j + 2*img.cols -1)/3;  
        theta = theta*CV_PI/180 - CV_PI/2;
    }
    else if(c == 'r')
    {
        waypoint_i = (ver_i + 2*img.rows -1)/3;
        waypoint_j = (ver_j)/3;
       
        theta = theta*CV_PI/180 + CV_PI/2;
    }
    
    NavPoint way_point;
    way_point.x = waypoint_j;
    way_point.y = waypoint_i;
    way_point.angle = theta;
    return way_point;
}

#endif