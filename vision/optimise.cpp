// IGVC filteration.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include"opencv2/highgui/highgui.hpp"
#include"opencv2/imgproc/imgproc.hpp"
#include"opencv2/core/core.hpp"
#include<iostream>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <string>

using namespace std;
using namespace cv;

#define alpha 50

#define threshb 30
#define threshg 50
#define threshr 70

#define threshbW 200
#define threshgW 200
#define threshrW 200

int max (int a, int b)
{
	return (a>b)?a:b;
}

int min (int a, int b)
{
	return (a<b)?a:b;
}

float mapped (float a, float ist, float iend, float ost, float oend)
{
	return (a-ist)*(oend-ost)/(iend-ist) + ost;
}

int value(Mat &img, int i, int j)
{
	float b = img.at<Vec3b>(i, j)[0];
	float g = img.at<Vec3b>(i, j)[1];
	float r = img.at<Vec3b>(i, j)[2];
	return max(max(r, g), b);
}

int lowvalue(Mat &img, int i, int j)
{
	float b = img.at<Vec3b>(i, j)[0];
	float g = img.at<Vec3b>(i, j)[1];
	float r = img.at<Vec3b>(i, j)[2];
	return min(min(r, g), b);
}

VideoCapture vid("IGVC.mp4");

int variance(int i, int j, Mat *img)
{
	int b = img->at<Vec3b>(i, j)[0];
	int g = img->at<Vec3b>(i, j)[1];
	int r = img->at<Vec3b>(i, j)[2];
	int avg = (b + g + r) / 3;
	int x = ((b*b) + (g*g) + (r*r)) / 3;
	return (x - (avg*avg));
}

void variableThresh(Mat &img, Mat &out, int ker, int init, int final, int res)
{
	Mat piecein, pieceout;
	int increment=(final-init)/res;
	int val=final+increment;
	for (int i = 0; i < img.rows; i+=img.rows/res)
	{
		Rect r(0, i, img.cols, img.rows/res);
		piecein = img(r);
		adaptiveThreshold(piecein, pieceout, 255, ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY, ker, val-=increment);
		pieceout.copyTo(out(r));
	}
}

int frame_width = vid.get(CV_CAP_PROP_FRAME_WIDTH);
int frame_height = vid.get(CV_CAP_PROP_FRAME_HEIGHT);

int main()
{
	namedWindow("fulfinal",WINDOW_NORMAL);
	namedWindow("img", 0);
	VideoWriter video("optimised.avi",CV_FOURCC('M','J','P','G'), 15, Size(frame_width,frame_height));
	int i=0;
	int num=0;
	while (1)
	{
		Mat img;// = imread("colorbarrels2.png");
		vid >> img;
		if (i%2==0)
		{
			Mat img2(img.rows, img.cols, CV_8UC3, Scalar(0));		//blue
			Mat img3(img.rows, img.cols, CV_8UC3, Scalar(0));		//red
			Mat img4(img.rows, img.cols, CV_8UC1, Scalar(0));		// blur blue
			for (int i = 0; i < img.rows; i++)
			{
				for (int j = 0; j < img.cols; j++)
				{
					(img2.at<Vec3b>(i, j)[0]) = (img.at<Vec3b>(i, j)[0]);		//blue
					(img2.at<Vec3b>(i, j)[1]) = img.at<Vec3b>(i, j)[1]+mapped(img.at<Vec3b>(i, j)[1], 0, 255, alpha, 0);		//green
					(img2.at<Vec3b>(i, j)[2]) = max(img.at<Vec3b>(i, j)[2]-mapped(img.at<Vec3b>(i, j)[2], 0, 255, alpha, 0), 0); 		//red2
					
					if (img2.at<Vec3b>(i, j)[1] == value(img2, i, j))
					{
						if (img2.at<Vec3b>(i, j)[0] == lowvalue(img2, i, j))
						{
							if (	
									!((img.at<Vec3b>(i, j)[0] < threshb || img.at<Vec3b>(i, j)[0] > threshbW) && 
									(img.at<Vec3b>(i, j)[1] < threshg || img.at<Vec3b>(i, j)[1] > threshgW) && 
									(img.at<Vec3b>(i, j)[2] < threshr || img.at<Vec3b>(i, j)[2] > threshrW) )
								)
							{
								continue;
							}
						}
					}
					img4.at<uchar>(i, j) = 255;
					img2.at<Vec3b>(i, j)[0] = img.at<Vec3b>(i, j)[0]*(255-img4.at<uchar>(i, j))/255;
					img2.at<Vec3b>(i, j)[1] = img.at<Vec3b>(i, j)[1]*(255-img4.at<uchar>(i, j))/255;
					img2.at<Vec3b>(i, j)[2] = img.at<Vec3b>(i, j)[2]*(255-img4.at<uchar>(i, j))/255;
				}
			}

			imshow("img", img);
			imshow("fulfinal", img2);
			waitKey(1);
			video.write(img2);
			i=0;
		}
		i++;
	}
	vid.release();
	video.release();
	destroyAllWindows();
	return 0;
}