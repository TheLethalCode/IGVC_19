#ifndef RANSAC
#define RANSAC


#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <bits/stdc++.h>

/*
   parabolas are fit assuming top left as origin - x towards right and y downwards
 */

using namespace std;
using namespace cv;

//structure to define the Parabola parameters
typedef struct Parabola
{
    int numModel = 0;
    float a1 = 0.0;
    float b1 = 0.0;
    float c1 = 0.0;
    float a2 = 0.0;
    float b2 = 0.0;
    float c2 = 0.0;
}Parabola;


// //set threshold for white color
// #define wTh 50
// //define no of iteration, max dist squre of pt from our estimated Parabola
// #define iteration 1000

// #define maxDist 300
// //define threshold distance to remove white pixel near lane1
// #define removeDist 300
// //define minimum number of points to be lie on a lane
// #define minLaneInlier 3000 // 2000 for night

// #define minPointsForRANSAC 500

Parabola swap(Parabola param) {

    float temp1, temp2, temp3;
    temp1=param.a1;
    temp2=param.b1;
    temp3=param.c1;
    param.a1=param.a2;
    param.b1=param.b2;
    param.c1=param.c2;
    param.a2=temp1;
    param.b2=temp2;
    param.c2=temp3;

    return param;
}

//calculation of Parabola parameters based on 3 randonmly selected points
float get_a(Point p1, Point p2, Point p3)
{
    int x1 = p1.x;
    int x2 = p2.x;
    int x3 = p3.x;
    int y1 = p1.y;
    int y2 = p2.y;
    int y3 = p3.y;
    float del = (y2 - y1)*(y3 - y2)*(y1 - y3);
    float del_a = (x2 - x1)*(y3 - y2) - (x3 - x2)*(y2 - y1);
    return(del_a / del);
}
float get_b(Point p1, Point p2, Point p3)
{
    int x1 = p1.x;
    int x2 = p2.x;
    int x3 = p3.x;
    int y1 = p1.y;
    int y2 = p2.y;
    int y3 = p3.y;
    float del = (y2 - y1)*(y3 - y2)*(y1 - y3);
    float del_b = (x3 - x2)*((y2*y2) - (y1*y1)) - (x2 - x1)*((y3*y3) - (y2*y2));
    return(del_b / del);
}
float get_c(Point p, float a, float b)
{
    int x = p.x;
    int y = p.y;
    return(x - (a*y*y) - (b*y));
}

float min(float a, float b)
{
	if(a<=b)
		return a;
	return b;
}

//calculation of error b/w actual and estimated y
float get_delX(Point p, float a, float b, float c)
{
    float predictedX = (a*(p.y*p.y) + b*p.y + c);
    float errorx=abs(p.x - predictedX);
    float y1,y2,errory;
    y1 = (-b - sqrt((b*b)-(4*a*(c-p.x))))/(2*a);
    y2 = (-b + sqrt((b*b)-(4*a*(c-p.x))))/(2*a);
    if(y1 < 0)
    	y1=10000000;
    if(y2 < 0)
    	y2=10000000;
    errory = min(abs(y1 - p.y),abs(y2 - p.y));
    return(min(errorx,errory));
}

//choose Parabola parameters of best fit curve basis on randomly selected 3 points
Parabola ransac(vector<Point> ptArray, Parabola param)
{
    int numDataPts = ptArray.size();
    int maxInlier = 0;
    Parabola tempParam;

    //start loop
    for(int i = 0; i < iteration; ++i)
    {
        int d1 = rand() % numDataPts;
        int d2 = rand() % numDataPts;
        int d3 = rand() % numDataPts;

        Point p1 = ptArray[d1];
        Point p2 = ptArray[d2];
        Point p3 = ptArray[d3];

        if(d1 == d2 || d2 == d3 || d1 == d3)
        {
            continue;
        }

        //get the Parabola parameters
        float temp_a = get_a(p1, p2, p3);
        float temp_b = get_b(p1, p2, p3);
        float temp_c = get_c(p1, temp_a, temp_b);

        //count inliers on predicted Parabola
        int tempInlier = 0;
        for(int j = 0; j < numDataPts; ++j)
        {
            Point z = ptArray[j];
            float error = get_delX(z, temp_a, temp_b, temp_c);
            if(error < maxDist)
            {
                ++tempInlier;
            }
        }

        if((tempInlier > maxInlier) || (i == 0))
        {
            maxInlier = tempInlier;
            tempParam.a1 = temp_a;
            tempParam.b1 = temp_b;
            tempParam.c1 = temp_c;
        }
    }
    //end of for loop


    if(maxInlier > minLaneInlier)
    {   
        cout << "maxInlier: " << maxInlier << endl;
        if(param.numModel == 0)
        {
            param.a1 = tempParam.a1;
            param.b1 = tempParam.b1;
            param.c1 = tempParam.c1;
        }
        else if(param.numModel == 1)
        {
            param.a2 = tempParam.a1;
            param.b2 = tempParam.b1;
            param.c2 = tempParam.c1;
        }
        param.numModel += 1;
    }
    return param;
}

//Check wheather a point is near lane1
bool IsNearLane1(Parabola param1, Point p)
{
    float dist = get_delX(p, param1.a1, param1.b1, param1.c1);
    if(dist < removeDist)
    {
        return true;
    }
    else
    {
        return false;
    }
}

float dist(Point A,Point B)
{
    return (sqrt(pow(A.x-B.x,2)+pow(A.y-B.y,2)));
}

Point centroid(float a,float b,float c,Mat img)
{
    Point A;
    int i,j,x,y;
    int sum_x = 0,sum_y = 0,count=0;

    for(j=0;j<img.rows;j++)
    {
        x=a*j*j+b*j+c;

        if(x>=0 && x<img.cols)
        {
            sum_y+=j;
            sum_x+=x;
            count++;
        }
    }

    A.x=sum_x/count;
    A.y=sum_y/count;

    return A;
}


//removes both the lanes if they intersect within the image frame
/*Parabola removeIntersectingLanes(Mat img, Parabola param) {

    float A = param.a1 - param.a2;
    float B = param.b1 - param.b2;
    float C = param.c1 - param.c2;

    if (B*B - 4*A*C < 0 || A == 0) {
        // cout << "Returning" << endl;
        return param;
    }

    float y_intersection1 = (-B - sqrt(B*B - 4*A*C))/(2*A);
    float y_intersection2 = (-B + sqrt(B*B - 4*A*C))/(2*A);

    if (y_intersection1 >= 0 && y_intersection1 < img.rows) {
        float x_intersection1 = (param.a1*y_intersection1*y_intersection1) + (param.b1*y_intersection1) + param.c1;

        if (x_intersection1 >= 0 && x_intersection1 < img.cols) {
            param.a1 = 0;
            param.b1 = 0;
            param.c1 = 0;

            param.a2 = 0;
            param.b2 = 0;
            param.c2 = 0;

            param.numModel = 0;

            cout << "Intersecting1 at: " << x_intersection1 << " " << y_intersection1 << endl;
        }
    }

    if (y_intersection2 >= 0 && y_intersection2 < img.rows) {
        float x_intersection2 = (param.a1*y_intersection2*y_intersection2) + (param.b1*y_intersection2) + param.c1;

        if (x_intersection2 >= 0 && x_intersection2 < img.cols) {
            param.a1 = 0;
            param.b1 = 0;
            param.c1 = 0;

            param.a2 = 0;
            param.b2 = 0;
            param.c2 = 0;

            param.numModel = 0;

            cout << "Intersecting2 at: " << x_intersection2 << " " << y_intersection2 << endl;
        }
    }
    return param;
}*/

Parabola getRansacModel(Mat img,Parabola previous)
{
    //apply ransac for first time it will converge for one lane
    vector<Point> ptArray1;
    Point A,B,C;

    for(int i = 0; i < img.rows; ++i)
    {
        for(int j = 0; j < img.cols; ++j)
        {
            int wVal = img.at<uchar>(i,j);
            if(wVal > wTh)
            {
                Point pt;
                pt.x = j;
                pt.y = i;
                ptArray1.push_back(pt);
            }
        }
    }

    //declare a Parabola vaiable to store the Parabola
    Parabola param;

    //get parameters of first Parabola form ransac function
    if(ptArray1.size() > minPointsForRANSAC)
    {
        param = ransac(ptArray1, param);
    }

    //Remove white pixel form image near lane1 and apply ransac to get lane2
    vector<Point> ptArray2;
    for(int i = 0; i < img.rows; ++i)
    {
        for(int j = 0; j < img.cols; ++j)
        {
            Point q;
            q.x = j;
            q.y = i;
            int wVal = img.at<uchar>(i,j);
            if(wVal > wTh)
            {
                if(!IsNearLane1(param, q))
                {
                    ptArray2.push_back(q);
                }
            }
        }
    }

    // param = removeIntersectingLanes(img, param);

    float temp1,temp2,temp3;
    //get parameters of second Parabola form ransac function
    if(ptArray2.size() > minPointsForRANSAC)
    {
        param = ransac(ptArray2, param);
    }


    //Lane classification based on previous frames
    
    //resolved intersection issue
    if(param.numModel==2)
    {
        for(int i=0;i<img.rows;i++)
        {
            int j1=param.a1*i*i+param.b1*i+param.c1;
            int j2=param.a2*i*i+param.b2*i+param.c2;
            if(j1>0&&j2>0&&j1<img.cols&&j2<img.cols&&fabs(j1-j2)<3)
            {
                return previous;
            }
        }   
    }

    //if two lanes
    if(param.numModel==2) {

        //set left and right according to x value at middle row
        if(param.a1*(img.rows/2)*(img.rows/2)+param.b1*(img.rows/2)+param.c1>param.a2*(img.rows/2)*(img.rows/2)+param.b2*(img.rows/2)+param.c2)
        {
            param = swap(param);
        }
    }

    //if one lane, assign same as previous frame if it had one lane
    if(param.numModel==1)
    {
        if(previous.numModel==1)
        {
            //if prev frame had right lane
            if(previous.a1==0&&previous.b1==0&&previous.c1==0)
            {
                //if current frame has left lane
                if(param.a2==0&&param.b2==0&&param.c2==0)
                {
                    param = swap(param);
                }
            }
            //if prev frame had left lane
            else if(previous.a2==0&&previous.b2==0&&previous.c2==0)
            {
                //if current frame has right lane
                if(param.a1==0&&param.b1==0&&param.c1==0)
                {
                    param = swap(param);
                }
            }
        }

        if(previous.numModel==2)
        {
            A=centroid(previous.a1,previous.b1,previous.c1,img);
            B=centroid(previous.a2,previous.b2,previous.c2,img);

            //if current frame has right lane
            if(param.a1==0&&param.b1==0&&param.c1==0)
            {
                C=centroid(param.a2,param.b2,param.c2,img);
                if(dist(A,C)<dist(B,C))
                {
                    param = swap(param);
                }
            }
            //if current frame has left lane
            else
            {
                C=centroid(param.a1,param.b1,param.c1,img);
                if(dist(A,C)>dist(B,C))
                {
                    param = swap(param);
                }
            }
        }
    }


    return param;
}

Mat drawLanes(Mat img, Parabola lanes) {

    Mat fitLanes(img.rows, img.cols, CV_8UC3, Scalar(0,0,0));

    vector<Point2f> left_lane, right_lane;
    float a1 = lanes.a1, a2 = lanes.a2, b1 = lanes.b1, b2 = lanes.b2, c1 = lanes.c1, c2 = lanes.c2;

    for (float y = 0; y < fitLanes.rows; y++){

        float x;
        if (a1 != 0 && b1 != 0 && c1 != 0) {
            x = a1*y*y + b1*y + c1;
            left_lane.push_back(Point2f(x, y));
        }

        if (a2 != 0 && b2 != 0 && c2 != 0) {
            x = a2*y*y + b2*y + c2;
            right_lane.push_back(Point2f(x, y));
        }

    }

    Mat left_curve(left_lane, true);
    left_curve.convertTo(left_curve, CV_32S); //adapt type for polylines
    polylines(fitLanes, left_curve, false, Scalar(255, 0, 0), 3, CV_AA);

    Mat right_curve(right_lane, true);
    right_curve.convertTo(right_curve, CV_32S); //adapt type for polylines
    polylines(fitLanes, right_curve, false, Scalar(0, 0, 255), 3, CV_AA);

    return fitLanes;
}


#endif
