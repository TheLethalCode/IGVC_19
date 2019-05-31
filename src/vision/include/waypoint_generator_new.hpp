#ifndef WAY_POINT_GENERATION
#define WAY_POINT_GENERATION

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <math.h>
#include "ransac_new_2.hpp"


#define ll 80   //x co-ordinate shift   
#define bb 80   //y co-ordinate shift
#define dist 180

using namespace std;
using namespace cv;

/*
This waypoint generation uses the RANSAC parabola model[y^2=a(x-c) (BOTTOM-LEFT origin)]
    to the equation of the form x= ay^2 + by + c (TOP-LEFT origin)

On applying the conversion, the eqn. becomes (img.rows-y)^2= a(x-c)

NOTE: 
    * theta is 0-180 in CCW sense (-ve -> +ve x-axis)
*/

//For this we use the following struct storing 6 variables. 
typedef struct Parabola2 {
    int numModel = 0;
    float a1 = 0.0;
    float c1 = 0.0;
    float a2 = 0.0;
    float b2 = 0.0;
    float b1 = 0.0;
    float c2 = 0.0;
} Parabola2;

float xc,yc;

// Custom struct for storing x, y & yaw
struct NavPoint{
    int x;
    int y;
    float angle;
};

/*
Function to find the lane closest to a point    
    * Checks it by finding closeness of (ay^2 + by + c -x) to 0
    * Returns: 
        ~ 1 if input point closer to left lane 
        ~ 2 if close to right lane
        ~ 0 if none
*/
int checklane(int y,int x,Mat img,Parabola2 lanes)
{
    if(fabs(lanes.a1*y*y+lanes.b1*y+lanes.c1-x)< (30/4)) 
        return 1;

    if(fabs(lanes.a2*y*y+lanes.b2*y+lanes.c2-x)< (30/4))
        return 2;
    
    return 0;
}

//Ensures that the chosen pixel is not out of bounds of the image
bool isValid(Mat img, int i, int j) 
{
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

            if((img.at<uchar>(i,j)==255))
                
                return 0;
        }
    }
    return 1;
}

//returns the angle assuming angle along -ve y axis as 0 and cloclwise to it as -ve and anticlockwise as +ve
float GetAngle(Mat img,int min,int max,Parabola2 lanes,float xc, float yc)
{
 // cout<<"min: "<<min<<" max: "<<max<<endl;
 float th1=0,th2=0;
    if(lanes.numModel==2)
    {
        th1=atan(2*lanes.a1*yc+lanes.b1);
        th2=atan(2*lanes.a2*yc+lanes.b2);
        return ((th1+th2)/2);

    }
    else if(lanes.numModel==1)
    {
        if(max<20)
            return 0;
        else if(min>160)
            return 180;
         if(lanes.a2==0)
         {
            th1=atan(2*lanes.a1*yc+lanes.b1);
            return th1;
         }
         else
         {
            th2=atan(2*lanes.a2*yc+lanes.b2);
            return th2;
         }


    }
    else
    {
        return 0;
    }
}

//Signum Function
int sgn(float x){
    if(x>0) return 1;
    else if(x<0) return -1;
    else return 0;
}

//calculates the angle bounds of the left and right lanes
void GetAngleBounds (Mat img,int *min,int *max,Parabola2 lanes)
{
    int theta,theta_min=0,theta_max=180;
    float theta_rad;
    
    //theta is 0-180 in CCW sense (-ve -> +ve x-axis)
    for(theta=0;theta<180;theta++)
    {
        theta_rad=theta*CV_PI/180;

        /*
        stepsize: Distance between 2 waypoints (dynamic_reconfigure parameter)
            * We find the lane on which the point along the given angle at 'stepsize' distance is on. 
            * Frame: LIDAR frame (bottom-middle origin)
        */
        if(checklane(img.rows-stepsize*sin(theta_rad),img.cols/2-stepsize*cos(theta_rad),img,lanes)==1)
        {
            theta_min=theta;
            // break;
        }

        if(checklane(img.rows-stepsize*sin(theta_rad),img.cols/2-stepsize*cos(theta_rad),img,lanes)==2)
        {
            theta_max=theta;
//-----------------------------------------------------------------------------------------------        
// Why the break neccessary?
            break;  
        }
    }

    *min=theta_min;
    *max=theta_max;
}

/*
Gets the position of waypt. in the LIDAR frame (BOTTOM-MIDDLE origin):

NOTE:   
    * Initial theta convention. 
    * the Mat img contains only RANSAC plotted lanes.
*/
NavPoint getCoordinatesxy(Mat img,int *theta_min,int *theta_max,Parabola2 lanes)
{
    int i,j;
    int theta;
    NavPoint pt;
    float ptx,pty;
    pt.x=img.cols/2;
    pt.y=img.rows/2;

    GetAngleBounds(img,theta_min,theta_max,lanes);
    int theta_mid=((*theta_min)+(*theta_max))/2;
    
    //BOTH LANES PRESENT
    if(lanes.numModel==2)   
    {
        // Intuition
        j=img.rows*2/5;

        /* Checking if given y co-ordinate's corresponding x co-ordinates 
         of the 2 curves are within image boudaries. */
        if((lanes.a1*j*j+lanes.b1*j+lanes.c1)>0 && (lanes.a2*j*j+lanes.b2*j+lanes.c2)<img.cols)
        {
            pt.y=j;
            //x= mean of x co-ordinates of 2 curves for the given y co-ordinate.
            pt.x= ((lanes.a1*j*j+lanes.b1*j+lanes.c1)+(lanes.a2*j*j+lanes.b2*j+lanes.c2))/2;
        }
    }

    //SINGLE LANE PRESENT
    else if(lanes.numModel==1)
    {
        Parabola2 temp;
        float theta,theta_m,m;
        // float bottom = lanes.a1*(img.rows*img.rows) + lanes.b1 * img.rows + lanes.c1,top = lanes.c1,right = (-lanes.b1-math.sqrt(lanes.b1*lanes.b1 ))
        
        //LEFT LANE
        if(lanes.a1!=0||lanes.b1!=0||lanes.c1!=0)
        {
            /*
            To get the waypoint in a direction parallel to the current lane,
                we create a shifted curve.
            i.e. the eqn. now becomes x-ll = a(y-bb)^2 + b(y-bb) + c
            */  
            temp.a1 = lanes.a1;
//-------------------------------------------------------------------------------------------------
//Still doubtful           
            /*
            Signum is being used to accomodate for the 2 concavities(i.e. +ve/-ve 'a' values)
                * If we shift for waypt. by maths (i.e. a(y-bb)^2 + b(y-bb) + c= x-ll),
                    we'll get the shift in the direction of the concavity.  
                * Signum automatically takes care of this. 
            */ 
            temp.b1=lanes.b1-2*lanes.a1*bb*sgn(lanes.a1);
            temp.c1 = lanes.c1+ll+lanes.a1*bb*bb-sgn(lanes.a1)*bb*lanes.b1;
//-------------------------------------------------------------------------------------------------

            float theta_rad;
            
            /*
            Finding the max angle from left that can be acoomodated in the stepsize.
            Hence the start from 180.
            */
            for(theta=180;theta>0;theta--)
            {
                theta_rad=theta*CV_PI/180;

                if(checklane(img.rows-stepsize*sin(theta_rad),img.cols/2-stepsize*cos(theta_rad),img,temp)==1)
                    theta_m=theta;

            }

            theta_rad=theta_m*CV_PI/180;    //Converting to radians
            
            //Creating a pt. at theta angle & stepsize away
            pt.x = img.cols/2-stepsize*cos(theta_rad);
            pt.y = img.rows-stepsize*sin(theta_rad);
            
            /*
            If theta is less than 45, 
                the waypoint will be generated too close to the bottom.
            */
            if(theta_m<45)
            {
                
                float xd,yd;
                xd = (img.cols/2)-stepsize*cos(theta_rad);
                yd = img.rows - stepsize*sin(theta_rad);
                
                // So we find a new theta by moving along it from the shifted point.
                // A new stepsize ('dist' variable) is used.
//-------------------------------------------------------------------------------------------------
                
//-------------------------------------------------------------------------------------------------
                
                for(theta=0;theta<180;theta++)
                {
                    theta_rad=theta*CV_PI/180;

                    if(checklane(yd-dist*sin(theta_rad),xd-dist*cos(theta_rad),img,temp)==1)
                        theta_m=theta;
                }
                theta_rad = theta_m*CV_PI/180;
                pt.x = xd-dist*cos(theta_rad);
                pt.y = yd-dist*sin(theta_rad);
            }


        }

        //RIGHT LANE
        else if(lanes.a2!=0||lanes.b2!=0||lanes.c2!=0)
        {
            temp.a2 = lanes.a2;
            temp.b2=lanes.b2 +2*lanes.a2*bb*sgn(lanes.a2);
            temp.c2 = lanes.c2-ll+lanes.a2*bb*bb+sgn(lanes.a2)*bb*lanes.b2;
            float theta_rad;
            
            // For the max angle from right, we start from 0.
            for(theta=0;theta<180;theta++)
            {
                theta_rad=theta*CV_PI/180;
                
                if(checklane(img.rows-dist*sin(theta_rad),img.cols/2-stepsize*cos(theta_rad),img,temp)==2)
                {
                    theta_m=theta;
                    break;
                }

            }
            theta_rad=theta_m*CV_PI/180;
            
            //Creating a pt. at theta angle & stepsize away
            pt.x = img.cols/2-stepsize*cos(theta_rad);
            pt.y = img.rows-stepsize*sin(theta_rad);
            
            /*
            If theta is greater than 135, 
                the waypoint will be generated too close to the bottom.
            */
            if(theta>135)
            {
                float xd,yd;
                xd = (img.cols/2)-stepsize*cos(theta_rad);
                yd = img.rows - stepsize*sin(theta_rad);
                
                // So we find a new theta by increasing the stepsize using dist variable.
                for(theta=0;theta<180;theta++)
                {
                    theta_rad=theta*CV_PI/180;

                    if(checklane(yd-dist*sin(theta_rad),xd-dist*cos(theta_rad),img,temp)==1)
                    {
                        theta_m=theta;
                        //cout<<"theta_min : "<<theta_min<<endl;
                    }

                }
                theta_rad = theta_m*CV_PI/180;
                pt.x = xd-dist*cos(theta_rad);
                pt.y = yd-dist*sin(theta_rad);
            }
        }
    }

    if(lanes.numModel==1)
    {
        if(*theta_max<25)
           {
                pt.x=img.cols/2+stepsize/2;
                pt.y=img.rows;
           } 
        if(*theta_min>155)
        {
            pt.x=img.cols/2-stepsize/2;
            pt.y=img.rows;
        }
    }
      return pt;
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
    //It has got classified lanes(left is 1st curve)
    Parabola2 lanes;
    int count_check;
    float a1 = lan.a1;
    float a2 = lan.a2;
    float c1 = lan.c1;
    float c2 = lan.c2;
    lanes.numModel=lan.numModel;

    //If no left lane
    if(a1==0)
    {
        lanes.a1 = 0;
        lanes.b1 = 0;
        lanes.c1 = 0;
    }
    else
    {
        //Equation at line 22    
        lanes.a1 = 1/a1;
        lanes.b1 = (-2*img.rows)/a1;
        lanes.c1 = (img.rows*img.rows + a1*c1)/a1;
        
        //if in case parabola is linearly fit into x= my + c
        if(fabs(lanes.a1)<0.00001)
            lanes.c1=c1;
    }

    //If no right lane
    if(a2==0)
    {
        //Equation at line 22
        lanes.a2 = 0;
        lanes.b2 = 0;
        lanes.c2 = 0;
    }
    else
    {
        lanes.a2 = 1/a2;
        lanes.b2 = (-2*img.rows)/a2;
        lanes.c2 = (img.rows*img.rows + a2*c2)/a2;
        
        //if in case parabola is linearly fit into x= my + c
        if(fabs(lanes.a2)<0.00001)
            lanes.c2=c2;
    }


    //Plotting transformed image
    if(false) {
    Mat fitLanes1 = drawLanes1(img, lanes);
    namedWindow("Waypoint RANSAC plot",0);
    imshow("Waypoint RANSAC plot",fitLanes1);
    }

    NavPoint way_point;
    int theta_min,theta_max;

    way_point= getCoordinatesxy(img,&theta_min,&theta_max,lanes) ;
    float slope = GetAngle(img,theta_min,theta_max,lanes,way_point.x,way_point.y);

    way_point.angle = slope;
    
    count_check=0;
    while(!isValid_point(img,way_point.y,way_point.x)&&count_check<6)
    {   
        way_point.x-=10*sin(slope);
        way_point.y-=10*cos(slope);
        count_check++;
    }


    return way_point;
}

//Plots waypoint in a given binary image from a 'NavPoint' message
Mat plotWaypoint(Mat costmap, NavPoint waypoint_image) 
{
    Point origin = Point(waypoint_image.x-1, waypoint_image.y-1);   
    // -1 added to avoid seg fault if img.rows/cols returned
    float x = origin.x - 25*cos(CV_PI/2 - waypoint_image.angle);
    float y = origin.y - 25*sin(CV_PI/2 - waypoint_image.angle);
    Point dest = Point(x,y);
    
    //Drawing waypoint accordingly in the costmap image 
    circle(costmap, origin, 5, Scalar(255), -1, 8, 0);
    arrowedLine(costmap, origin, dest, Scalar(255), 3, 8, 0, 0.1);

    return costmap;
}

#endif