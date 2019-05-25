#pragma once
#include"opencv2/highgui/highgui.hpp"
#include"opencv2/imgproc/imgproc.hpp"
#include"opencv2/core/core.hpp"
#include<iostream>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

void brightest(Mat &img, int brightthresh) //brightest pixels below brightthresh will be neglected (reduces noise)
{
	Mat Simg1, Simg2, Simg3;

	//Gaussian blur on image
	GaussianBlur(img, Simg1, Size(5, 5), 0, 0);
	
	//dividing the image in half vertically and storing in two images
	Simg2 = Simg1(Rect(Simg1.cols / 2, 0, Simg1.cols / 2, Simg1.rows));
	Simg3 = Simg1(Rect(0, 0, Simg1.cols / 2, Simg1.rows));
	double min = 0, max = 0;
	Point minLoc, maxLoc;
	Mat Simg4(img.rows, img.cols, CV_8UC1, Scalar(0));

	//finding brightest pixel per row of each half
	for (int i = 0; i < Simg2.rows; i++)
	{
		//maxLoc contains coordinate of maximum value
		//minLoc contains coordinate of minimum value
		minMaxLoc(Simg2.row(i), &min, &max, &minLoc, &maxLoc);
		if (max > brightthresh)
		circle(Simg4, Point(maxLoc.x + Simg1.cols / 2, i), 1, Scalar(255), 1);
	}

	for (int i = 0; i < Simg2.rows; i++)
	{
		//maxLoc contains coordinate of maximum value
		//minLoc contains coordinate of minimum value
		minMaxLoc(Simg3.row(i), &min, &max, &minLoc, &maxLoc);
		if (max > brightthresh)
		circle(Simg4, Point(maxLoc.x, i), 1, Scalar(255), 1);
	}

	//updating original image
	Simg4.copyTo(img);
}