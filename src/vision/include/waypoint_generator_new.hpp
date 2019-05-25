#ifndef WAY_POINT_GENERATION
#define WAY_POINT_GENERATION

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <math.h>
#include "ransac_new.hpp"

using namespace std;
using namespace cv;

/*#define pixelsPerMeter 112.412
#define stepsize 3.5*pixelsPerMeter

#define botlength 90
#define botwidth 30*/

typedef struct Parabola2 {
    int numModel = 0;
    float a1 = 0.0;
    float c1 = 0.0;
    float a2 = 0.0;
    float b2 = 0.0;
    float b1 = 0.0;
    float c2 = 0.0;
} Parabola2;

struct NavPoint{
    int x;
    int y;
    float angle;
};

//returns 1 if input point closer to left lane, 2 if close to right lane, 0 if none
int checklane(int y,int x,Mat img,Parabola2 lanes)
{
    if(fabs(lanes.a1*y*y+lanes.b1*y+lanes.c1-x)< (30/4)) {
        cout<<"1 :"<<endl;
        return 1;
    }
    if(fabs(lanes.a2*y*y+lanes.b2*y+lanes.c2-x)< (30/4)) {
        cout<<"2 :"<<endl;
        return 2;
    }
    return 0;
}

bool isValid(Mat img, int i, int j) {
    if (i < 0 || i >= img.rows || j < 0 || j >= img.cols) {
        return false;
    }
    return true;
}

//check a circle of radius such that the bot can move in that circle without hitting any lanes or obstacles
int isValid_point(Mat img, int i, int j)
{
    float r;
    float r1 = sqrt(pow(botwidth/2,2)+pow(botlength/2,2));

    int x1,y1;
    for(r=r1+1;r>0;r--)
    {
        for(int theta = 0; theta <360;theta++)
        {
            x1=j+r*cos((float)theta*CV_PI/180);
            y1=i-r*sin((float)theta*CV_PI/180);

            if (!isValid(img, y1, x1)) {
                return 0;
            }

            if((img.at<uchar>(y1,x1)==255))
                return 0;
        }
    }
    return 1;
}

//returns the angle assuming angle along -ve y axis as 0 and cloclwise to it as -ve and anticlockwise as +ve
float GetAngle(Mat img,int min,int max,Parabola2 lanes,float coordinateAngle)
{
 cout<<"min: "<<min<<" max: "<<max<<endl;
 float th1=0,th2=0;
    if(lanes.numModel==2)
    {
        th1=atan(2*lanes.a1*(img.rows-stepsize*sin(coordinateAngle))+lanes.b1);
        th2=atan(2*lanes.a2*(img.rows-stepsize*sin(coordinateAngle))+lanes.b2);
        return ((th1+th2)/2);

    }
    else if(lanes.numModel==1)
    {
         if(lanes.a2==0)
         {
            th1=atan(2*lanes.a1*(img.rows-stepsize*sin(coordinateAngle))+lanes.b1);
            return th1;
         }
         else
         {
            th2=atan(2*lanes.a2*(img.rows-stepsize*sin(coordinateAngle))+lanes.b2);
            return th2;
         }


    }
    else
    {
        return 0;
    }

   /* float min_rad=min*CV_PI/180; //varies from 0 to PI taking -ve x axis as 0 angle, positive clockwise
    float max_rad=max*CV_PI/180;

    float a1 = lanes.a1, a2 = lanes.a2, b1 = lanes.b1, b2 = lanes.b2, c1 = lanes.c1, c2 = lanes.c2;
   
    //edited
    float angle1=0;
    float angle2=0;

    // here the average of the slope of lanes is used to approximate the heading of waypoint angle
    // if two lanes are there then the average of both the lanes is used finally

    // ##################### changes done
    // earlier the value is updated considering slope_up started from the stepsize to top of the image
    // changing that to zero the average considering whole lane

    //numModel contains the number of lanes
    //if number of lanes is 2 then update both angles
    if(lanes.numModel==2)
    {  
        for(int slope_up=0; slope_up<img.rows; slope_up++)
        {
            angle1+=atan((2*a1*(img.rows-slope_up)+b1));
            angle2+=atan((2*a2*(img.rows-slope_up)+b2));
        }
    }
    // if number of lanes is 1 then update only one angle  
    else if(lanes.numModel==1)
    {
        if(a1 == 0 && b1 == 0 && c1 == 0)
            for(int slope_up=0; slope_up<img.rows; slope_up++)
            {
                // angle1+=atan(1/(2*a1*(img.rows-slope_up)+b1));
                angle2+=atan((2*a2*(img.rows-slope_up)+b2));
            }
        if(a2 == 0 && b2 == 0 && c2 == 0)
            for(int slope_up = 0; slope_up<img.rows; slope_up++)
            {
                 angle1+=atan((2*a1*(img.rows-slope_up)+b1));
                // angle2+=atan(1/(2*a2*(slope_up-slope_up)+b2));
            }
    }
    angle1/=(img.rows);
    angle2/=(img.rows);

    //editing end

    //removed_prev

    // float angle1 = atan(1/(2*a1*delYmin + b1));
    // float angle2 = atan(1/(2*a2*delYmax + b2));

    cout << "Left lane: " << angle1*180/CV_PI << endl;
    cout << "Right lane: " << angle2*180/CV_PI << endl;

    if(lanes.numModel==2)
    {
        float d = (angle1 + angle2)/2;
        return d;
    }
    else if(lanes.numModel==1)
    {
        float d;
        if(angle1==0)
            d = angle2;
        else
            d=angle1;
        return d;
    }
    // else if(min!=0&&max==180)
    // {
    //     float d = angle1;
    //     return d;
    // }
    else return 0;*/

}

//calculates the angle bounds of the left and right lanes
void GetAngleBounds (Mat img,int *min,int *max,Parabola2 lanes)
{
    int theta,theta_min=0,theta_max=180;
    float theta_rad;
    for(theta=0;theta<180;theta++)
    {
        theta_rad=theta*CV_PI/180;

        if(checklane(img.rows-stepsize*sin(theta_rad),img.cols/2-stepsize*cos(theta_rad),img,lanes)==1)
        {
            theta_min=theta;
            //cout<<"theta_min : "<<theta_min<<endl;
        }

        if(checklane(img.rows-stepsize*sin(theta_rad),img.cols/2-stepsize*cos(theta_rad),img,lanes)==2)
        {
            theta_max=theta;
            //cout<<"THETA_MAX PRINT: "<<theta_max<<endl;
            break;
        }
    }



    *min=theta_min;
    *max=theta_max;
}

//gets the angle assuming the bottom center as origin taking clockwise angle as positive, -ve x axis as 0 degree line
int getCoordinateAngle(Mat img,int *theta_min,int *theta_max,Parabola2 lanes)
{
    int i,j;
    int theta;
    int theta_head=90;

    //img contains both obs and lanes
    GetAngleBounds(img,theta_min,theta_max,lanes);
    int theta_mid=((*theta_min)+(*theta_max))/2;
    cout<<"theta min : "<<*theta_min<<" theta max : "<<*theta_max<<endl;

    for(theta=0;theta<((*theta_max)-(*theta_min))/2;theta++)
    {
        i=img.rows-stepsize*sin((theta_mid+theta)*CV_PI/180);
        j=img.cols/2-stepsize*cos((theta_mid+theta)*CV_PI/180);
        if (!isValid(img, i, j)) {
            continue;
        }
        if(img.at<Vec3b>(i,j)[0]==0&&img.at<Vec3b>(i,j)[1]==0&&img.at<Vec3b>(i,j)[2]==0)
        {
            if(isValid_point(img,i,j))
            {
                theta_head=theta_mid+theta;
                break;
            }
        }

        i=img.rows-stepsize*sin((theta_mid-theta)*CV_PI/180);
        j=img.cols/2-stepsize*cos((theta_mid-theta)*CV_PI/180);
        if (!isValid(img, i, j)) {
            continue;
        }
        if(img.at<Vec3b>(i,j)[0]==0&&img.at<Vec3b>(i,j)[1]==0&&img.at<Vec3b>(i,j)[2]==0)
        {
            if(isValid_point(img,i,j))
            {
                theta_head=theta_mid-theta;
                break;
            }
        }
    }

    return theta_head;
}

//To plot transformed image in waypoint from ransac
 Mat drawLanes1(Mat topView, Parabola2 lanes) {

    Mat fitLanes(topView.rows, topView.cols, CV_8UC3, Scalar(0,0,0));

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

NavPoint find_waypoint(Parabola lan,Mat img)
{
    Parabola2 lanes;
    float a1 = lan.a1;
    float a2 = lan.a2;
    float c1 = lan.c1;
    float c2 = lan.c2;
    cout<<"no. of lanes: "<<lan.numModel<<endl;
    lanes.numModel=lan.numModel;


    if(a1==0)
    {
        lanes.a1 = 0;
        lanes.b1 = 0;
        lanes.c1 = 0;
    }
    else
    {
        lanes.a1 = 1/a1;
        lanes.b1 = (-2*img.rows)/a1;
        lanes.c1 = (img.rows*img.rows + a1*c1)/a1;
        if(fabs(lanes.a1)<0.00001)
            lanes.c1=c1;
    }
    cout<<"a1: "<<lanes.a1<<" b1: "<<lanes.b1<<" c1: "<<lanes.c1<<endl;

    if(a2==0)
    {
        lanes.a2 = 0;
        lanes.b2 = 0;
        lanes.c2 = 0;
    }
    else
    {
        lanes.a2 = 1/a2;
        lanes.b2 = (-2*img.rows)/a2;
        lanes.c2 = (img.rows*img.rows + a2*c2)/a2;
        if(fabs(lanes.a2)<0.00001)
            lanes.c2=c2;
    }




    //Plotting transformed image to check
    if(true) {
    Mat fitLanes1 = drawLanes1(img, lanes);
    namedWindow("Waypoint RANSAC plot",0);
    imshow("Waypoint RANSAC plot",fitLanes1);
    }

    NavPoint way_point;
    int theta_min,theta_max;

    float coordinateAngle = getCoordinateAngle(img,&theta_min,&theta_max,lanes) * CV_PI/180;
    float slope = GetAngle(img,theta_min,theta_max,lanes,coordinateAngle);

    way_point.x = (img.cols/2-stepsize*cos(coordinateAngle));
    way_point.y = (img.rows-stepsize*sin(coordinateAngle));
    cout<<"coordinate angle "<<coordinateAngle<<endl;
    way_point.angle = slope;

    return way_point;
}

Mat plotWaypoint(Mat costmap, NavPoint waypoint_image) {
    Point origin = Point(waypoint_image.x-1, waypoint_image.y -1);
    float x = origin.x - 25*cos(CV_PI/2 - waypoint_image.angle);
    float y = origin.y - 25*sin(CV_PI/2 - waypoint_image.angle);
    Point dest = Point(x,y);
    circle(costmap, origin, 5, Scalar(120), -1, 8, 0);
    arrowedLine(costmap, origin, dest, Scalar(120), 3, 8, 0, 0.1);

    return costmap;
}

#endif