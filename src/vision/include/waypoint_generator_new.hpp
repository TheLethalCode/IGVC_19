#ifndef WAY_POINT_GENERATION
#define WAY_POINT_GENERATION

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <math.h>
#include "ransac_new_2.hpp"
#define ll 80
#define bb 80
#define dist 180

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
float xc,yc;
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

            if((img.at<uchar>(i,j)==255))
                
                return 0;
        }
    }
    return 1;
}

//returns the angle assuming angle along -ve y axis as 0 and cloclwise to it as -ve and anticlockwise as +ve
float GetAngle(Mat img,int min,int max,Parabola2 lanes,float xc, float yc)
{
 cout<<"min: "<<min<<" max: "<<max<<endl;
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
NavPoint getCoordinatesxy(Mat img,int *theta_min,int *theta_max,Parabola2 lanes)
{
    int i,j,check1=0;
    int theta;
    int theta_head=90;
    NavPoint pt;
    float ptx,pty;
    pt.x=img.cols/2;
    pt.y=img.rows/2;

    //img contains both obs and lanes
    GetAngleBounds(img,theta_min,theta_max,lanes);
    int theta_mid=((*theta_min)+(*theta_max))/2;
    cout<<"theta min : "<<*theta_min<<" theta max : "<<*theta_max<<endl;
    /*if(*theta_max<35||*theta_min>145)
        stepsize=stepsize/5;*/
    if(lanes.numModel==2)
    {
        /*for(theta=0;theta<((*theta_max)-(*theta_min))/2;theta++)
        {
            i=img.rows-stepsize*sin((theta_mid+theta)*CV_PI/180);
            j=img.cols/2-stepsize*cos((theta_mid+theta)*CV_PI/180);
            
            if (!isValid(img, i, j)) {
                continue;
            }
            if(img.at<Vec3b>(i,j)[0]==0&&img.at<Vec3b>(i,j)[1]==0&&img.at<Vec3b>(i,j)[2]==0)
            {
                cout  << isValid(img, i, j) <<endl;
                if(isValid_point(img,i,j))
                {
                    pt.y=i;
                    ptx=j;
                    pty=i;
                    pt.x=j;
                    check1=1;
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
                    pt.y=i;
                    ptx=j;
                    ptx=i;
                    pt.x=j;
                    check1=1;
                    break;
                }
            }
        }*/
        j=img.rows*2/5;
        j=img.rows*2/5;
        if((lanes.a1*j*j+lanes.b1*j+lanes.c1)>0&&(lanes.a2*j*j+lanes.b2*j+lanes.c2)<img.cols)
        {
            pt.y=j;
            pt.x= ((lanes.a1*j*j+lanes.b1*j+lanes.c1)+(lanes.a2*j*j+lanes.b2*j+lanes.c2))/2;

        }
    }
    else if(lanes.numModel==1)
    {
        Parabola2 temp;
        float theta,theta_m,m;
       // float bottom = lanes.a1*(img.rows*img.rows) + lanes.b1 * img.rows + lanes.c1,top = lanes.c1,right = (-lanes.b1-math.sqrt(lanes.b1*lanes.b1 ))
       // if(lanes.c1>0&&lanes.c1<img.cols){
         //   if(lanes)
        
        if(lanes.a1!=0||lanes.b1!=0||lanes.c1!=0)
        {
            temp.a1 = lanes.a1;
            temp.b1=lanes.b1-2*lanes.a1*bb*sgn(lanes.a1);
            temp.c1 = lanes.c1+ll+lanes.a1*bb*bb-sgn(lanes.a1)*bb*lanes.b1;
            float theta_rad;
            for(theta=180;theta>0;theta--)
            {
                theta_rad=theta*CV_PI/180;

                if(checklane(img.rows-stepsize*sin(theta_rad),img.cols/2-stepsize*cos(theta_rad),img,temp)==1)
                {
                    theta_m=theta;
                    //cout<<"theta_min : "<<theta_min<<endl;
                }

            }
            theta_rad=theta_m*CV_PI/180;
            pt.x = img.cols/2-stepsize*cos(theta_rad);
            pt.y = img.rows-stepsize*sin(theta_rad);
            if(theta_m<45)
            {
                float xd,yd;
                cout << "noooooooooooooooooooooooooooooooooooooooooooooooooooooooooooo" << endl;
                xd = (img.cols/2)-stepsize*cos(theta_rad);
                yd = img.rows - stepsize*sin(theta_rad);
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
        else if(lanes.a2!=0||lanes.b2!=0||lanes.c2!=0)
        {
            temp.a2 = lanes.a2;
            temp.b2=lanes.b2 +2*lanes.a2*bb*sgn(lanes.a2);
            temp.c2 = lanes.c2-ll+lanes.a2*bb*bb+sgn(lanes.a2)*bb*lanes.b2;
            float theta_rad;
            for(theta=0;theta<180;theta++)
            {
                theta_rad=theta*CV_PI/180;
                
                if(checklane(img.rows-dist*sin(theta_rad),img.cols/2-stepsize*cos(theta_rad),img,temp)==2)
                {
                    theta_m=theta;
                    break;
                    //cout<<"theta_min : "<<theta_min<<endl;
                }

            }
            theta_rad=theta_m*CV_PI/180;
            pt.x = img.cols/2-stepsize*cos(theta_rad);
            pt.y = img.rows-stepsize*sin(theta_rad);
            if(theta>135)
            {
                cout << "yeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeees" <<endl;
                float xd,yd;
                xd = (img.cols/2)-stepsize*cos(theta_rad);
                yd = img.rows - stepsize*sin(theta_rad);
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
    //if(*theta_max==180&&*theta_min!=0)
    //{

        

    //}
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
    cout<<"point is "<<pt.x<<" pt.y:"<<pt.y<<endl;
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

    Parabola2 lanes;
    int count_check;
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
    if(false) {
    Mat fitLanes1 = drawLanes1(img, lanes);
    namedWindow("Waypoint RANSAC plot",0);
    imshow("Waypoint RANSAC plot",fitLanes1);
    }

    NavPoint way_point;
    int theta_min,theta_max;

    way_point= getCoordinatesxy(img,&theta_min,&theta_max,lanes) ;
     /*way_point.x = (img.cols/2-stepsize*cos(coordinateAngle));
    way_point.y = (img.rows-stepsize*sin(coordinateAngle));*/
    float slope = GetAngle(img,theta_min,theta_max,lanes,way_point.x,way_point.y);

   
    /*cout<<"coordinate angle "<<coordinateAngle<<endl;*/
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

Mat plotWaypoint(Mat costmap, NavPoint waypoint_image) {
    Point origin = Point(waypoint_image.x-1, waypoint_image.y -1);
    float x = origin.x - 25*cos(CV_PI/2 - waypoint_image.angle);
    float y = origin.y - 25*sin(CV_PI/2 - waypoint_image.angle);
    Point dest = Point(x,y);
    circle(costmap, origin, 5, Scalar(255), -1, 8, 0);
    arrowedLine(costmap, origin, dest, Scalar(255), 3, 8, 0, 0.1);

    return costmap;
}

#endif