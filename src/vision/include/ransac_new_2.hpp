#ifndef RANSAC_NEW_2
#define RANSAC_NEW_2


#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <bits/stdc++.h>

/*
    * This RANSAC model fits both lane curves simultaneously by taking 4 random points (2 on each lanes).
    * The parabolas are fit assuming bottom-left as origin: i.e. x towards right and y downwards.
    * The equation used for parabola is (y^2)= a(x-c).
    * Both a & c are found by solving simultaneous linear equations.
 */
int debug=0;
int change= 100;    //for sudden change function
using namespace std;
using namespace cv;

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
} Parabola;

Parabola swap(Parabola param) {

    float temp1, temp3;
    temp1=param.a1;
    temp3=param.c1;

    param.a1=param.a2;
    param.c1=param.c2;

    param.a2=temp1;
    param.c2=temp3;

    return param;
}

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
        if(a1*c1<0 && a1*(img.cols-c1)>0)
        {
            float y1=sqrt(-1.0*a1*c1);
            float y2=sqrt(a1*(img.cols-c1));

            if(y1>(2*img.rows)/5 && y1<(3*img.rows)/5 && y2>(2*img.rows)/5 && y2<(3*img.rows)/5)
            {
                return previous;
            }

        }

        if(a2*c2<0 && a2*(img.cols-c2)>0)
        {
            float y1=sqrt(-1.0*a2*c2);
            float y2=sqrt(a2*(img.cols-c2));

            if(y1>(2*img.rows)/5 && y1<(3*img.rows)/5 && y2>(2*img.rows)/5 && y2<(3*img.rows)/5)
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
    return (sqrt(pow(A.x-B.x,2)+pow(A.y-B.y,2)));
}


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

/*
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
*/

//choose Parabola parameters of best fit curve basis on randomly selected 4 points
Parabola ransac(vector<Point> ptArray, Parabola param, Mat img, Parabola previous)
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

        // check if the centroids of the 2 parabolas are too close.
        if (dist(centroid(tempParam.a1,tempParam.c1,img),centroid(tempParam.a2,tempParam.c2,img)) < 20.0){
            //cout<<"centroid issue."<<endl;
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

        //Checking if the curves are too close at the bottom
        else if(fabs(tempParam.c1 - tempParam.c2) < 40.0){
            //cout<<"c1-c2 issue. "<<fabs(tempParam.c1 - tempParam.c2)<<endl;
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
        else if ((score_common/(score_common + score_l_loc + score_r_loc+1))*100 > common_inliers_thresh) {
            //cout<<"common points issue."<<endl;
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
    cout  <<"score_l_gl : "<<score_l_gl<<" score_r_gl : "<<score_r_gl<<endl;

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

    //Checking for no sudden change
    // bestTempParam = no_sudden_change(bestTempParam, img, previous);
    
    if(true){
        // cout<<"bestTempParam.numModel : "<<bestTempParam.numModel<<endl;
        // cout<<"bestTempParam.a1 : "<<bestTempParam.a1<<" bestTempParam.c1 : "<<bestTempParam.c1<<endl;
        // cout<<"bestTempParam.a2 : "<<bestTempParam.a2<<" bestTempParam.c2 : "<<bestTempParam.c2<<endl;
    }
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



Parabola getRansacModel(Mat img,Parabola previous)
{
    //apply ransac for first time it will converge for one lane

    //Vector for storing white points
    vector<Point> ptArray1;

    if(debug!=0)
    { 
        //cout << "---------------------------------NEW RANSAC----------------------------------" << endl;
    }
    // if (grid_white_thresh >= grid_size*grid_size) {
    //     grid_white_thresh = grid_size*grid_size -1;
    // }

    Mat plot_grid(img.rows,img.cols,CV_8UC1,Scalar(0));


    int count = 0;
    //Iterating through all the grif elements
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


    namedWindow("grid",WINDOW_NORMAL);
    
    imshow("grid",plot_grid);



    //declare a Parabola vaiable to store the Parabola
    Parabola param;
    //get parameters of first Parabola form ransac function

    //Checking if we have sufficient no. of points to initialise RANSAC
    if(ptArray1.size() > minPointsForRANSAC )
    {
        return_prev_params = 1;
        param = ransac(ptArray1, param, img, previous);

    }

    else {
        return previous;
    }
    //Lane classification based on previous frames
    //if two lanes

    /*if(v.size() < 6)
    {
        v.push_back(param);
    }
    else
    {
        vector<Parabola>::iterator it; 
        it = v.begin(); 
        v.erase(it); 
    }
    Parabola final;
    float div=0;
    for(int i=0; i<v.size(); i++) 
    {
        final.a1+= v[i].a1/(pow(2,i));
        final.c1+= v[i].c1/(pow(2,i));
        final.a2+= v[i].a2/(pow(2,i));
        final.c2+= v[i].c2/(pow(2,i));
        div+= (1/pow(2,i));
    }
    final.a1=final.a1/div;
    final.a2=final.a2/div;
    final.c1=final.c1/div;
    final.c2=final.c2/div;
    final.numModel= final.numModel/div;

    if(final.numModel >= 1.5) final.numModel = 2;
    else final.numModel = 1;*/
    // for(int i=0; i<ptArray1.size(); i++)
    // {
    //     float dist_l = get_del(ptArray1[i], param.a1, param.c1);
    //     float dist_r = get_del(ptArray1[i], param.a2, param.c2);
    //     if(dist_l < maxDist || dist_r < maxDist)
    //     {
    //         img.at<uchar>(img.rows - ptArray1[i].y, ptArray1[i].x) = 255;
    //     }       
    // }

    param=classify_lanes(img,param,previous);

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
