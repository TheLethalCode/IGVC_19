#pragma once
#include"opencv2/highgui/highgui.hpp"
#include"opencv2/imgproc/imgproc.hpp"
#include"opencv2/core/core.hpp"
#include<iostream>
#include <opencv2/opencv.hpp>
#include <string>

using namespace std;
using namespace cv;

#define fac 2.2
#define ObsThresh 30
#define stripelen 70
#define lanewid 100
#define whitethresh 150 
#define minarea 250
#define brightpixelthresh 200

int max(int a, int b)
{
	return (a > b) ? a : b;
}

int min(int a, int b)
{
	return (a < b) ? a : b;
}

float mapped(float a, float ist, float iend, float ost, float oend)
{
	return (a - ist)*(oend - ost) / (iend - ist) + ost;
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

float mod(Point pt)
{
	float val = pt.x*pt.x + pt.y*pt.y;
	if (val < 0)
	{
		cout << 1;
	}
	return sqrt(val);
}
typedef Point3_<uint8_t> Pixel;
void blurContour(const vector<Point>& src, vector<Point>& dst, float k)
{
	int siz = src.size();
	int off = (k - 1) / 2;
	for (int i = 0; i < siz; i++)
	{
		Point sum(0, 0);
		for (int j = 0; j < k; j++)
		{
			sum += src[(i + j) % siz];
			if ((i + j) % siz < 0 || (i + j) % siz > siz)
			{
				cout << 1;
			}
		}
		dst.push_back(sum / k);
	}
	if (dst[0] == dst[siz - 1])
	{
		cout << 1;
	}
	//cout<<dst[siz-1];
}

Mat object_remove(Mat img1, int areathresh)
{
	int num = 0;
	Mat img3;
	Mat img2(img1.rows, img1.cols, CV_8UC1, Scalar(0));
	Mat img4(img1.rows, img1.cols, CV_8UC1, Scalar(255));
	//dilate(img2, img2, Mat(), Point(-1, -1), 1, 1, 1);
	vector<vector<Point> > contours;

	vector<Vec4i> hierarchy;
	findContours(img1, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);

	for (size_t i = 0; i < contours.size(); i++)
	{
		double area = contourArea(contours[i]);
		if (area < areathresh)
		{
			contours.erase(contours.begin() + i);
			i--;
			//	cout<<i<<endl;
		}
	}

	hierarchy.clear();

	int contsize = contours.size();
	vector<vector<Point> > simple(contsize);

	for (int i = 0; i < contsize; i++)
	{
		drawContours(img4, contours, i, Scalar(0), -40, 8, vector<Vec4i>(), 0, Point());
		drawContours(img4, contours, i, Scalar(0), 10, 8, vector<Vec4i>(), 0, Point());
	}

	contours.clear();
	simple.clear();

	return img4;
}

void removeStripes(Mat &Simg1, int stripewid)
{
	//Mat Simg1(img.rows, img.cols, CV_8UC1, Scalar(0));
	Mat Simg2(Simg1.rows, Simg1.cols, CV_8UC1, Scalar(0));
	Mat Simg3(Simg1.rows, Simg1.cols, CV_8UC1, Scalar(0));
	Simg1.copyTo(Simg2);
	//img.copyTo(Simg1);
	//img.copyTo(Simg2);
	for (int i = 0; i < Simg1.cols; i++)
	{
		int lastwhite = 0;
		for (int j = 0; j < Simg1.rows - 1; j++)
		{
			if (Simg1.at<uint8_t>(j + 1, i) < Simg1.at<uint8_t>( j, i))
			{
				lastwhite = j;
				//circle(Simg2, Point(lastblack, i), 3, Scalar(255), 3);
			}
			else if (Simg1.at<uint8_t>( j + 1, i) > Simg1.at<uint8_t>( j, i))
			{
				//cout << j - lastblack << endl;
				if (j - lastwhite < stripewid)
				{
					line(Simg2, Point(i, lastwhite), Point(i, j), Scalar(255), 2);
					//circle(Simg2, Point(lastblack, i), j - lastblack, Scalar(255), 3);
					//cout << 1;
				}
			}
		}
	}
	//Simg2 = object_remove(Simg2);
	Simg1 = Simg2.clone();
	//Simg1.copyTo(Simg3, Simg2);
	//Simg3.copyTo(Simg1);
}

void brightest(Mat &img, int thresh)
{
	Mat Simg1, Simg2, Simg3;
//	cvtColor(img, Simg1, COLOR_BGR2GRAY);
	GaussianBlur(img, Simg1, Size(5, 5), 0, 0);
	Simg2 = Simg1(Rect(Simg1.cols / 2, 0, Simg1.cols / 2, Simg1.rows));
	Simg3 = Simg1(Rect(0, 0, Simg1.cols / 2, Simg1.rows));
	double min = 0, max = 0;
	Point minLoc, maxLoc;
	Mat Simg4(img.rows, img.cols, CV_8UC1, Scalar(0));
	for (int i = 0; i < Simg2.rows; i++)
	{
		//Mat row = Simg1.row(i);

		//maxLoc contains coordinate of maximum value
		minMaxLoc(Simg2.row(i), &min, &max, &minLoc, &maxLoc);
		//cout << maxLoc;
		if (max > thresh)
		circle(Simg4, Point(maxLoc.x + Simg1.cols / 2, i), 1, Scalar(255), 2);
	}

	for (int i = 0; i < Simg2.rows; i++)
	{
		//Mat row = Simg1.row(i);

		//maxLoc contains coordinate of maximum value
		minMaxLoc(Simg3.row(i), &min, &max, &minLoc, &maxLoc);
		//cout << maxLoc;
		if (max > thresh)
		circle(Simg4, Point(maxLoc.x, i), 1, Scalar(255), 2);
	}

	Simg4.copyTo(img);
}

void removeWhite(Mat &img, int lanewid, int whithresh, int minarea)
{
	Mat Simg1(img.rows, img.cols, CV_8UC1, Scalar(0));
	Mat Simg2(img.rows, img.cols, CV_8UC1, Scalar(0));
	Mat Simg3(img.rows, img.cols, CV_8UC1, Scalar(0));
	threshold(img, Simg1, whithresh, 255, THRESH_BINARY);
	erode(Simg1, Simg1, 3);
	//img.copyTo(Simg1);
	//img.copyTo(Simg2);
	for (int i = 0; i < img.rows; i++)
	{
		int lastblack = 0;
		for (int j = 0; j < img.cols - 1; j++)
		{
			if (Simg1.at<uint8_t>(i, j + 1) > Simg1.at<uint8_t>(i, j))
			{
				lastblack = j;
				//circle(Simg2, Point(lastblack, i), 3, Scalar(255), 3);
			}
			else if (Simg1.at<uint8_t>(i, j + 1) < Simg1.at<uint8_t>(i, j))
			{
				//cout << j - lastblack << endl;
				if (j - lastblack > lanewid)
				{
					line(Simg2, Point(lastblack, i), Point(j, i), Scalar(255), 2);
					//circle(Simg2, Point(lastblack, i), j - lastblack, Scalar(255), 3);
					//cout << 1;
				}
			}
		}
	}
	for (int i = 0; i < Simg1.cols; i++)
	{
		int lastblack = 0;
		for (int j = 0; j < Simg1.rows - 1; j++)
		{
			if (Simg1.at<uint8_t>(j + 1, i) > Simg1.at<uint8_t>(j, i))
			{
				lastblack = j;
				//circle(Simg2, Point(lastblack, i), 3, Scalar(255), 3);
			}
			else if (Simg1.at<uint8_t>(j + 1, i) < Simg1.at<uint8_t>(j, i))
			{
				//cout << j - lastblack << endl;
				if (j - lastblack < lanewid)
				{
					line(Simg2, Point(i, lastblack), Point(i, j), Scalar(0), 2);
					//circle(Simg2, Point(lastblack, i), j - lastblack, Scalar(255), 3);
					//cout << 1;
				}
			}
		}
	}
	Simg2 = object_remove(Simg2, minarea);
	//imshow("vertical stripes", Simg2);
	//img = Simg2.clone();
	img.copyTo(Simg3, Simg2);
	Simg3.copyTo(img);
}

void object_remove_2(Mat &img)
{

		Mat Simg1(img.rows, img.cols, CV_8UC1, Scalar(0));		//2b-g
		Mat Simg2(img.rows, img.cols, CV_8UC3, Scalar(0));		//blue
		Mat Simg3(img.rows, img.cols, CV_8UC1, Scalar(0));		//red
		Mat Simg4(img.rows, img.cols, CV_8UC1, Scalar(0));		// blur blue
		Mat Simg5(img.rows, img.cols, CV_8UC1, Scalar(0));		//blur red
		Mat Simg6(img.rows, img.cols, CV_8UC1, Scalar(0));		//
		Mat Simg7(img.rows, img.cols, CV_8UC1, Scalar(255));		//removing salt and pepper noise
		img.copyTo(Simg2);
		img.forEach<Pixel>([](Pixel &p, const int * position) -> void
		{
			p.x = min(max(float(p.x) - fac * float(p.y) + float(p.z), 0), 255);//float(p.x)*float(p.z)/255;
			p.y = p.x;
			p.z = p.x;
		});
		cvtColor(img, Simg3, COLOR_BGR2GRAY);
		threshold(Simg3, Simg3, ObsThresh, 255, THRESH_BINARY);
		medianBlur(Simg3, Simg3, 5);
		erode(Simg3, Simg3, 3, Point(-1, -1), 3);
		Simg3 = object_remove(Simg3, minarea);
		Simg2.forEach<Pixel>([](Pixel &p, const int * position) -> void
		{
			p.x = min(max(2 * p.x - p.y, 0), 255);//float(p.x)*float(p.z)/255;
			p.y = p.x;
			p.z = p.x;
		});
		cvtColor(Simg2, Simg1, COLOR_BGR2GRAY);
		threshold(Simg3, Simg3, 128, 255, THRESH_BINARY_INV);
		removeStripes(Simg3, stripelen);
		threshold(Simg3, Simg3, 128, 255, THRESH_BINARY_INV);
		Simg1.copyTo(Simg4, Simg3);
		removeWhite(Simg4, lanewid, whitethresh, minarea);
		imshow("white removal", Simg4);
		brightest(Simg4, brightpixelthresh);
		Simg4.copyTo(img);
		//imshow("semifinal", Simg4);
		//imwrite("IGVCwhite2.png", Simg4);
		//imshow("final", Simg5);
		//waitKey(1);
}