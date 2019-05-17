#ifndef GRID_HPP
#define GRID_HPP





#include <iostream>
#include <math.h>
#include <opencv2/opencv.hpp>
using namespace std;
using namespace cv;
#define PI 3.14159
double grid_average(Mat img,int a,int b,int grid_row_size,int grid_col_size)//average of all pixel values of grid
{
	a=a*grid_row_size;
	b=b*grid_col_size;
	double average=0;
	int i,j;
	for (i = a; i <a+grid_row_size; ++i)
        {
        	for (j = b; j <b+ grid_col_size; ++j)
        	{
        		average+=img.at<uchar>(i,j);        		
           	}
        }
    return average/(grid_col_size*grid_row_size);

}
double grid_average1(Mat img,int a,int b,int grid_row_size,int grid_col_size)//average of all non zero pixel values of grid
{
	a=a*grid_row_size;
	b=b*grid_col_size;
	double average=0;
	double counter=0;
	int i,j;
	for (i = a; i <a+grid_row_size; ++i)
        {
        	for (j = b; j <b+ grid_col_size; ++j)
        	{
        		if(img.at<uchar>(i,j)>90)
        		{
        			average+=img.at<uchar>(i,j);
        			counter++;
        		}        		
           	}
        }
    return (average/counter);

}
double grid_mark(Mat img,int a,int b,int grid_row_size,int grid_col_size)
{
	a=a*grid_row_size;
	b=b*grid_col_size;
	double average=0;
	int i,j;
    
	for (i = a; i <a+grid_row_size; ++i)
    {
        img.at<Vec3b>(i,b+grid_col_size-1)[0]=0; 
        img.at<Vec3b>(i,b+grid_col_size-1)[1]=255; 
        img.at<Vec3b>(i,b+grid_col_size-1)[2]=255;
    }
    for (j = b; j <b+ grid_col_size; ++j)
    {
        img.at<Vec3b>(a+grid_row_size-1,j)[0]=0; 
        img.at<Vec3b>(a+grid_row_size-1,j)[1]=255; 
        img.at<Vec3b>(a+grid_row_size-1,j)[2]=255;       		
    }

}
void grid_set_zero(Mat img,int a,int b,int grid_row_size,int grid_col_size)
{
	a=a*grid_row_size;
	b=b*grid_col_size;
	int i,j;
    for (i = a; i <a+grid_row_size; ++i)
        {
        	for (j = b; j <b+ grid_col_size; ++j)
        	{
        		img.at<uchar>(i,j)=0; 
        		        		
           	}
        }


	

}
void grid_set_white(Mat img,int a,int b,int grid_row_size,int grid_col_size)
{
    a=a*grid_row_size;
    b=b*grid_col_size;
    int i,j;
    for (i = a; i <a+grid_row_size; ++i)
        {
            for (j = b; j <b+ grid_col_size; ++j)
            {
                img.at<uchar>(i,j)=255; 
                                
            }
        }


    

}

double grid_standard_deviation(Mat img,int a,int b,int grid_row_size,int grid_col_size,double average)
{
	a=a*grid_row_size;
	b=b*grid_col_size;
	double variance=0;
	int counter=0;
	int i,j;
	for (i = a; i <a+grid_row_size; ++i)
        {
        	for (j = b; j <b+ grid_col_size; ++j)
        	{
        		if(img.at<uchar>(i,j)>170)
        		{
        			counter++;
        			variance+=(img.at<uchar>(i,j)-average)*(img.at<uchar>(i,j)-average);

        		}        		
           	}
        }
    return sqrt(variance/counter);

}

void mark_with_grid(Mat img,int grid_row_size,int grid_col_size)
{
        int grid_row=img.rows/grid_row_size;
        int grid_col=img.cols/grid_col_size;
        int i,j;
        for (i = 0; i <grid_row; ++i)
        {
            for (j = 0; j < grid_col; ++j)
            {
                 grid_mark(img,i,j,grid_row_size,grid_col_size);

            }
            
        }
        imshow("Grid",img);

}
void grid_threshold(Mat img,int grid_row_size,int grid_col_size,int threshold_avg_grid,int standard_deviation,int max_pixel)
{
    int i,j;
    int grid_row=img.rows/grid_row_size;
    int grid_col=img.cols/grid_col_size;
    for (i = 0; i <img.rows; ++i)//preprocessing and setting all small pixel values to zero
    {
        for (j = 0; j < img.cols; ++j)
        {
            if((img.at<uchar>(i,j))<10)
                img.at<uchar>(i,j)=0;
        }       
    }
    for (i = 0; i <grid_row; ++i)
    {
        for (j = 0; j < grid_col; ++j)
        {
            if (grid_average(img,i,j,grid_row_size,grid_col_size)<threshold_avg_grid) 
            {
                grid_set_zero(img,i,j,grid_row_size,grid_col_size);
            }
            double temp=grid_average1(img,i,j,grid_row_size,grid_col_size);
            if (grid_standard_deviation(img,i,j,grid_row_size,grid_col_size,temp)<standard_deviation)
            {
                grid_set_zero(img,i,j,grid_row_size,grid_col_size);
            }
        }
    }
    for (i = 0; i <img.rows; ++i)
    {
        for (j = 0; j < img.cols; ++j)
        {
                if((img.at<uchar>(i,j))>max_pixel)
                    img.at<uchar>(i,j)=0;
        }   
    }
}




#endif