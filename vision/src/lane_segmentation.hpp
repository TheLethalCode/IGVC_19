#ifndef HEADER_FILE
#define HEADER_FILE

 


#include"opencv2/highgui/highgui.hpp"
#include"opencv2/imgproc/imgproc.hpp"
#include"opencv2/core/core.hpp"
#include<iostream>
#include<bits/stdc++.h>
#include<unistd.h>
using namespace cv;
using namespace std;
int variance(int i,int j,Mat *img)
{
	int b=img->at<Vec3b>(i,j)[0];
	int g=img->at<Vec3b>(i,j)[1];
	int r=img->at<Vec3b>(i,j)[2];
	int avg=(b+g+r)/3;
	int x=((b*b)+(g*g)+(r*r))/3;
	return (x-(avg*avg));
}
Mat oneblue(Mat img)
{
	Mat img2(img.rows,img.cols,CV_8UC1,Scalar(0));		//blue
	Mat img4(img.rows,img.cols,CV_8UC1,Scalar(0));		// blur blue
	Mat img6(img.rows,img.cols,CV_8UC1,Scalar(0));		//binary image
	Mat img7(img.rows,img.cols,CV_8UC1,Scalar(0));		//removing salt and pepper noise
	for(int i=0;i<img.rows;i++)
	{
		for(int j=0;j<img.cols;j++)
		{
			img2.at<uchar>(i,j)=img.at<Vec3b>(i,j)[0];
		}	
	}
	
	return img2;
}
Mat twoblueonegreen(Mat img)
{
	Mat img1(img.rows,img.cols,CV_8UC1,Scalar(0));		//2b-g
	Mat img2(img.rows,img.cols,CV_8UC1,Scalar(0));		//blue
	Mat img7(img.rows,img.cols,CV_8UC1,Scalar(0));		//green
	Mat img9(img.rows,img.cols,CV_8UC1,Scalar(0));		//final output
	for(int i=0;i<img.rows;i++)
	{
		for(int j=0;j<img.cols;j++)
		{
			img2.at<uchar>(i,j)=img.at<Vec3b>(i,j)[0];
		}	
	}
	for(int i=0;i<img.rows;i++)
	{
		for(int j=0;j<img.cols;j++)
		{
			img7.at<uchar>(i,j)=img.at<Vec3b>(i,j)[1];
		}	
	}
	for(int i=0;i<img.rows;i++)
	{
		for(int j=0;j<img.cols;j++)
		{
			if(((2*img2.at<uchar>(i,j))-img7.at<uchar>(i,j))<0)
				img1.at<uchar>(i,j)=0;
			else if(((2*img2.at<uchar>(i,j))-img7.at<uchar>(i,j))>255)
				img1.at<uchar>(i,j)=255;
			else
				img1.at<uchar>(i,j)=(2*img2.at<uchar>(i,j))-img7.at<uchar>(i,j);
		}	
	}
	return img1;
}
Mat twoblueonered(Mat img)
{
	Mat img1(img.rows,img.cols,CV_8UC1,Scalar(0));		//2b-r
	Mat img2(img.rows,img.cols,CV_8UC1,Scalar(0));		//blue
	Mat img7(img.rows,img.cols,CV_8UC1,Scalar(0));		//red
	Mat img9(img.rows,img.cols,CV_8UC1,Scalar(0));		//final output
	for(int i=0;i<img.rows;i++)
	{
		for(int j=0;j<img.cols;j++)
		{
			img2.at<uchar>(i,j)=img.at<Vec3b>(i,j)[0];
		}	
	}
	for(int i=0;i<img.rows;i++)
	{
		for(int j=0;j<img.cols;j++)
		{
			img7.at<uchar>(i,j)=img.at<Vec3b>(i,j)[2];
		}	
	}
	for(int i=0;i<img.rows;i++)
	{
		for(int j=0;j<img.cols;j++)
		{
			if(((2*img2.at<uchar>(i,j))-img7.at<uchar>(i,j))<0)
				img1.at<uchar>(i,j)=0;
			else if(((2*img2.at<uchar>(i,j))-img7.at<uchar>(i,j))>255)
				img1.at<uchar>(i,j)=255;
			else
				img1.at<uchar>(i,j)=(2*img2.at<uchar>(i,j))-img7.at<uchar>(i,j);
		}	
	}
	
	return img1;
}

#endif