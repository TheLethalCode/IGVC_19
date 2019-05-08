#include <iostream>
#include <math.h>
#include <opencv2/opencv.hpp>
#include "mlesac.hpp"
using namespace std;
using namespace cv;
#define PI 3.14159

Mat frame;
Mat grass;
Mat roi;
Mat filter;
void ROI()
{
    Mat temp(((9*frame.rows)/13), (frame.cols), CV_8UC3, Scalar(0,0,0));
    for (int i = 0; i < (9*frame.rows)/13; ++i)
    {
    	for (int j = 0; j < (frame.cols); ++j)
    	{


    		temp.at<Vec3b>(i,j)[0]=frame.at<Vec3b>(((2.8*frame.rows)/13)+i,j)[0];
    		temp.at<Vec3b>(i,j)[1]=frame.at<Vec3b>(((2.8*frame.rows)/13)+i,j)[1];
    		temp.at<Vec3b>(i,j)[2]=frame.at<Vec3b>(((2.8*frame.rows)/13)+i,j)[2];
    		
    	}

    }
    roi=temp.clone();
    
}

void grass_rm()
{
	Mat temp(roi.rows,roi.cols,CV_8UC1,Scalar(0));
	int i,j;

	for (i = 0; i < roi.rows; ++i)
	{
		for (j=0; j < roi.cols; ++j)
		{
			int pixel=2*roi.at<Vec3b>(i,j)[0]-roi.at<Vec3b>(i,j)[2];
			if (pixel<=0)
			{
				temp.at<uchar>(i,j)=0;
			}
			else if (pixel>=255)
			{
				temp.at<uchar>(i,j)=255;
			}
			else
			{
				temp.at<uchar>(i,j)=pixel;
			}
		}
	}
	grass=temp.clone();
}
double grid_average(Mat img,int a,int b,int grid_row_size,int grid_col_size)
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
double grid_mark(Mat img,int a,int b,int grid_row_size,int grid_col_size)
{
	a=a*grid_row_size;
	b=b*grid_col_size;
	double average=0;
	int i,j;
    for (i = a; i <a+grid_row_size; ++i)
        {
        	for (j = b; j <b+ grid_col_size; ++j)
        	{
        		img.at<Vec3b>(i,j)[0]=grass.at<uchar>(i,j); 
        		img.at<Vec3b>(i,j)[1]=grass.at<uchar>(i,j); 
        		img.at<Vec3b>(i,j)[2]=grass.at<uchar>(i,j);        		
           	}
        }


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
	double average=0;
	int i,j;
    for (i = a; i <a+grid_row_size; ++i)
        {
        	for (j = b; j <b+ grid_col_size; ++j)
        	{
        		img.at<uchar>(i,j)=0; 
        		        		
           	}
        }


	

}
void bird_eye_view()
{
    int frameWidth = roi.cols;
    int frameHeight = roi.rows;
    // mat container to receive images
    Mat source, destination;
    source=roi.clone();
    


    int alpha_ = 35, beta_ = 90, gamma_ = 90;
    int f_ = 500, dist_ = 500;

  


  

    resize(source, source,Size(frameWidth, frameHeight));

    double focalLength, dist, alpha, beta, gamma; 

    alpha =((double)alpha_ -90) * PI/180;
    beta =((double)beta_ -90) * PI/180;
    gamma =((double)gamma_ -90) * PI/180;
    focalLength = (double)f_;
    dist = (double)dist_;

    Size image_size = source.size();
    double w = (double)image_size.width, h = (double)image_size.height;


    // Projecion matrix 2D -> 3D
    Mat A1 = (Mat_<float>(4, 3)<< 
      1, 0, -w/2,
      0, 1, -h/2,
      0, 0, 0,
      0, 0, 1 );

  
    // Rotation matrices Rx, Ry, Rz

    Mat RX = (Mat_<float>(4, 4) << 
      1, 0, 0, 0,
      0, cos(alpha), -sin(alpha), 0,
      0, sin(alpha), cos(alpha), 0,
      0, 0, 0, 1 );

    Mat RY = (Mat_<float>(4, 4) << 
      cos(beta), 0, -sin(beta), 0,
      0, 1, 0, 0,
      sin(beta), 0, cos(beta), 0,
      0, 0, 0, 1  );

    Mat RZ = (Mat_<float>(4, 4) << 
      cos(gamma), -sin(gamma), 0, 0,
      sin(gamma), cos(gamma), 0, 0,
      0, 0, 1, 0,
      0, 0, 0, 1  );


    // R - rotation matrix
    Mat R = RX * RY * RZ;

    // T - translation matrix
    Mat T = (Mat_<float>(4, 4) << 
      1, 0, 0, 0,  
      0, 1, 0, 0,  
      0, 0, 1, dist,  
      0, 0, 0, 1); 
    
    // K - intrinsic matrix 
    Mat K = (Mat_<float>(3, 4) << 
      focalLength, 0, w/2, 0,
      0, focalLength, h/2, 0,
      0, 0, 1, 0
      ); 


    Mat transformationMat = K * (T * (R * A1));

    warpPerspective(source, destination, transformationMat, image_size, INTER_CUBIC | WARP_INVERSE_MAP);

    roi=destination.clone();
}
int main()
{
	VideoCapture vid("vid.mp4",0);
	int grid_row_size=15,grid_col_size=15;


	while(1)
	{
		vid>>frame;
	    ROI();
	    //bird_eye_view();

	    grass_rm();
	    Mat visual(roi.rows, roi.cols, CV_8UC3, Scalar(0,0,0));
	    int grid_row=grass.rows/grid_row_size;
	    int grid_col=grass.cols/grid_col_size;
	    int i,j;
	    for (i = 0; i <grass.rows; ++i)
        {
        	for (j = 0; j < grass.cols; ++j)
        	{
        		if((grass.at<uchar>(i,j))>200)
        			grass.at<uchar>(i,j)=0;
        	}
        	
        }
        for (i = 0; i <grid_row; ++i)
        {
        	for (j = 0; j < grid_col; ++j)
        	{
        		 if (grid_average(grass,i,j,grid_row_size,grid_col_size)<47)
        		 {
        		 	grid_set_zero(grass,i,j,grid_row_size,grid_col_size);
        		 }
        		 grid_mark(visual,i,j,grid_row_size,grid_col_size);

        	}
        	
        }
        
        // model lane=getMlesacModel(grass);
        // for (i = 0; i <roi.rows; ++i)
        // {
        	
        // 	j=lane.a1*i*i+lane.b1*i+lane.c1;
        // 	if(j>=0)
        // 	if(j<visual.cols) 
        // 	visual.at<Vec3b>(i,j)[0]=255;
        	
        // 	j=lane.a2*i*i+lane.b2*i+lane.c2;
        // 	if(j>=0)
        // 	if(j<visual.cols) 
        // 	visual.at<Vec3b>(i,j)[0]=255;

        	
        // }
		namedWindow("2b-g",CV_WINDOW_NORMAL);
		imshow("2b-g",grass);
		imshow("grid",visual);
	    imshow("wid",roi);
		waitKey(0);

	}

	return 0;




}