#ifndef LANE_FILTER_H
#define LANE_FILTER_H


#include <iostream>
#include <math.h>
#include <opencv2/opencv.hpp>
#define PI 3.14159
using namespace std;
using namespace cv;

double gaussian(int x,double sigma);
double **getTemplate(double sigma,double theta,int rows,int cols,double multiplier2,double multiplier1);
void visualization(double **matrix,int n,int m);
void print_matrix(double **matrix,int rows,int cols);
double sum_of_kernel(double **matrix,int rows,int cols);
//Mat apply_kernel(Mat img,double **matrix,int rows,int cols,int threshold2=0,int threshold3=255);

Mat apply_kernel(Mat img,double **matrix,int rows,int cols,int threshold2=0,int threshold3=255)
{
	int i,j,x,y;
	double sum=0;
	double sum_kernel=sum_of_kernel(matrix,rows,cols);
    int cols_mid=(cols-1)/2,row_mid=(rows-1)/2;
	Mat filter(img.rows,img.cols,CV_8UC1,Scalar(0));
	for(i=cols_mid;i<img.cols-cols_mid;i++)
	{
		for(j=row_mid;j<img.rows-row_mid;j++)
		{
			
			for(x=0;x<cols;x++)
			{
				for(y=0;y<rows;y++)
				{
					sum+=matrix[y][x]*(img.at<uchar>(j-row_mid+y,i-cols_mid+x));
				}
			}
		
			if(sum<threshold2)
				sum=0;
			else if((sum/sum_kernel)>threshold3)
				sum=255;
			
			//cout<<i<<" "<<j<<endl;
			   filter.at<uchar>(j,i)=sum/sum_kernel;
			sum=0;
			//cout<<"test"<<endl;
			//cout<<endl;
		}
	}
	return filter;
	
}

double gaussian(int x,double sigma)
{
	double r=pow((x/sigma),2);
	double value= (r-1)*exp(-1*r/2);///(1.414*3.14*pow(sigma,3));
	return value;
}
double **getTemplate(double sigma,double theta,int rows,int cols,double multiplier2=1,double multiplier1=1)
{
	int i,j;
	double val;
	double** matrix = new double*[rows];
    for (int i = 0; i < rows; ++i)
    	matrix[i] = new double[cols];
    int cols_mid=(cols-1)/2,row_mid=(rows-1)/2;
   
	for(i=0;i<cols;i++)
	{
		for(j=0;j<rows;j++)
		{
			val=-gaussian((cols_mid-i)*cos(PI*theta/180)-(j-row_mid)*sin(PI*theta/180),sigma);
			if(val<0)
				val*=multiplier2;
		    else
				val*=multiplier1;
			matrix[j][i]=val;
		}
	}
	return matrix;
}
void visualization(double **matrix,int n,int m)
{

	Mat visual(n,m,CV_8UC1,Scalar(0));
	int i,j;
	double max=0,ratio,min=matrix[0][0];
	for(i=0;i<m;i++)
	{
		for(j=0;j<n;j++)
		{
			if(matrix[j][i]>max)
				max=matrix[j][i];
			if(matrix[j][i]<min)
				min=matrix[j][i];
		}
	}
	max-=min;
	ratio=(double)255/max;
	for(i=0;i<m;i++)
	{
		for(j=0;j<n;j++)
		{
			matrix[j][i]-=min;
			matrix[j][i]*=255/max;
			visual.at<uchar>(j,i)=matrix[j][i];
		}
	}
	imshow("gaussian",visual);
	waitKey(0);
}
void print_matrix(double **matrix,int rows,int cols)
{
	int i,j=0;
	double sum = 0;
	for (j = 0; j < rows; ++j)
	{
		for(i=0;i<cols;i++)
		{	
			cout<<i<<" "<<j;
			cout<<matrix[j][i]<<" ";
		}
		cout<<endl;
	}
	
}
double sum_of_kernel(double **matrix,int rows,int cols)
{
	int i,j;
	double sum=0;
	for(i=0;i<cols;i++)
	{
		for(j=0;j<rows;j++)
		{
			sum+=matrix[j][i];
		}
	}
	return sum;
}

#endif

