#ifndef GRID_OBJECT_HPP
#define GRID_OBJECT_HPP

#include <iostream>
#include <math.h>
#include <opencv2/opencv.hpp>
#include "grid.hpp"
#include "basic.hpp"
#include "lane_segmentation.hpp"
//#include "combinedobs.hpp"
using namespace std;
using namespace cv;
#define PI 3.14159

Mat frame;
Mat grass;
Mat roi;
Mat filter;
Mat testing(Mat img)//return the 1.9*red-green-blue channel
{
    Mat img1(img.rows,img.cols,CV_8UC1,Scalar(0));      
    
    for(int i=0;i<img.rows;i++)
    {
        for(int j=0;j<img.cols;j++)
        {
            int pixel=1.9*img.at<Vec3b>(i,j)[2]-img.at<Vec3b>(i,j)[0]-img.at<Vec3b>(i,j)[1];
            if(pixel<0)
                img1.at<uchar>(i,j)=0;
            else if(pixel>255)
                img1.at<uchar>(i,j)=255;
            else
                img1.at<uchar>(i,j)=pixel;
        }   
    }
    return img1;
}
void grid_set_zero1(Mat img,int a,int b,int grid_row_size,int grid_col_size)//setting grid to 0 for a 3 channel image
{
    a=a*grid_row_size;
    b=b*grid_col_size;
    int i,j;
    for (i = a; i <a+grid_row_size; ++i)
        {
            for (j = b; j <b+ grid_col_size; ++j)
            {
                img.at<Vec3b>(i,j)[0]=0;
                img.at<Vec3b>(i,j)[1]=0;
                img.at<Vec3b>(i,j)[2]=0;
            }
        }
}
void smoothness_filter(Mat img,Mat frame,int grid_row_size,int grid_col_size,int standard_deviation)
{
    int i,j;
    int grid_row=img.rows/grid_row_size;
    int grid_col=img.cols/grid_col_size;
    for (i = 0; i <grid_row; ++i)
    {
        for (j = 0; j < grid_col; ++j)
        {
            double temp=grid_average1(img,i,j,grid_row_size,grid_col_size);
            if ((grid_standard_deviation(img,i,j,grid_row_size,grid_col_size,temp)<standard_deviation)) 
            {
                grid_set_white(frame,i,j,grid_row_size,grid_col_size);
                
            }
           
        }
    }
    
}
// void image_subtract1(Mat img,Mat obj)
// {
//     Mat temp(img.rows, img.cols, CV_8UC1, Scalar(0));
//     for (int i = 0; i < img.rows; ++i)
//     {
//         for (int j = 0; j < img.cols; ++j)
//         {
//             if (obj.at<uchar>(i,j))
//             {
//                 img.at<Vec3b>(i,j)[0]=0;
//                 img.at<Vec3b>(i,j)[1]=0;
//                 img.at<Vec3b>(i,j)[2]=0;
               
//             }
//         }
//     }
// }

void grid_obs(Mat frame,int grid_row_size,int grid_col_size,int standard_deviation_value )
{
    frame = ROI(frame,0,1,0,1);
    Mat orange =testing(frame);
    Mat b=oneblue(frame);
    Mat bg=twoblueonegreen(frame);
    Mat br=twoblueonered(frame);
    smoothness_filter(bg,orange,grid_row_size,grid_col_size,standard_deviation_value);
    //image_subtract1(frame,orange);
    imshow("win",orange);

}

#endif