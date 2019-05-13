#ifndef BASIC
#define BASIC

#include <iostream>
#include <math.h>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;




Mat image_subtract(Mat img,Mat obj)
{
    Mat temp(img.rows, img.cols, CV_8UC1, Scalar(0));
    for (int i = 0; i < img.rows; ++i)
    {
        for (int j = 0; j < img.cols; ++j)
        {
            if (!obj.at<uchar>(i,j))
            {
                temp.at<uchar>(i,j)=img.at<uchar>(i,j);
               
            }
        }
    }
    return temp;
}

Mat ROI(Mat input,double r1,double r2,double c1,double c2)
{
    Mat roi((r2-r1)*(input.rows), (c2-c1)*(input.cols), CV_8UC3, Scalar(0,0,0));
    for (int i = 0; i < roi.rows; ++i)
    {
    	for (int j = 0; j < roi.cols; ++j)
    	{
    		roi.at<Vec3b>(i,j)[0]=input.at<Vec3b>(input.rows*r1+i,input.cols*c1+j)[0];
    		roi.at<Vec3b>(i,j)[1]=input.at<Vec3b>(input.rows*r1+i,input.cols*c1+j)[1];
    		roi.at<Vec3b>(i,j)[2]=input.at<Vec3b>(input.rows*r1+i,input.cols*c1+j)[2];	
    	}
    }
    return roi;     
}
void add_image(Mat add,Mat img1,Mat img2,Mat img3)
{
	
	for (int i = 0; i < img1.rows; ++i)
    {
    	for (int j = 0; j < img1.cols; ++j)
    	{
    		if((img1.at<uchar>(i,j)==255)||(img2.at<uchar>(i,j)==255)||(img3.at<uchar>(i,j)==255))
    			add.at<uchar>(i,j)=255;
    	}
    }
    
}
Mat intersection(Mat img1,Mat img2,Mat img3)
{
	Mat final_intersection(img1.rows,img1.cols,CV_8UC1,Scalar(0));
	for (int i = 0; i < img1.rows; ++i)
    {
    	for (int j = 0; j < img1.cols; ++j)
    	{
    		if((img1.at<uchar>(i,j)==255)&&(img2.at<uchar>(i,j)==255)&&(img2.at<uchar>(i,j)==255))
    			final_intersection.at<uchar>(i,j)=255;
    	}
    }
    return final_intersection;
}
Mat convert_to_3_channel(Mat img)
{
    Mat img1(img.rows,img.cols,CV_8UC3,Scalar(0));
    for (int i = 0; i < img1.rows; ++i)
    {
        for (int j = 0; j < img1.cols; ++j)
        {
            img1.at<Vec3b>(i,j)[2]=img.at<uchar>(i,j);
            img1.at<Vec3b>(i,j)[1]=img.at<uchar>(i,j);
            img1.at<Vec3b>(i,j)[0]=img.at<uchar>(i,j);
        }
    }
    return img1;
    


}
Mat perspective_transform_3channel(Mat img)
{

    Mat top_view(img.rows,img.cols,CV_8UC3,Scalar(0,0,0));

    vector <Point2f> rect(4);
    vector <Point2f> lane1(4);
    //vector <Point2f> lane2(3);

    //points used to find homography matrix
    rect[3].x=440;
    rect[3].y=465;
    rect[2].x=440;
    rect[2].y=345;
    rect[1].x=717;
    rect[1].y=345;
    rect[0].x=717;
    rect[0].y=465;

    lane1[3].x=77;
    lane1[3].y=550-240;
    lane1[2].x=159;
    lane1[2].y=431-240;
    /*lane1[2].x=229;
    lane1[2].y=275;*/

    lane1[0].x=1132;
    lane1[0].y=510-240;
    lane1[1].x=1037;
    lane1[1].y=400-240;
    /*lane1[5].x=900;
    lane1[5].y=254;*/

    Mat h=findHomography(lane1,rect);

    // cout<<h<<endl;

    warpPerspective(img,top_view,h,top_view.size());

    return top_view;

}

#endif