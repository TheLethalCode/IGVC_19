
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
#define threshg 70
#define threshr 70

#define threshbW 210
#define threshgW 220
#define threshrW 210

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

typedef cv::Point3_<uint8_t> Pixel;

Mat &CreateObjectMask(Mat &src, Mat &dst) 
{
	src.copyTo(dst);
	dst.forEach<Pixel>([](Pixel &p, const int * position) -> void 
	{
	    Pixel img2;
	    bool flag = false;
	    (img2.x) = (p.x);		//blue
		(img2.y) = p.y+mapped(p.y, 0, 255, alpha, 0);		//green
		(img2.z) = max(p.z-mapped(p.z, 0, 255, alpha, 0), 0); 		//red2
		p = img2;
		if (img2.y == max(max(p.x, p.y), p.z)) 
		{
		 	if (img2.x == min(min(p.x, p.y), p.z))
		 	{
				if (	
						!((p.x < threshb || p.x > threshbW) && 
						(p.y < threshg || p.y > threshgW) && 
						(p.z < threshr || p.z > threshrW) )
					)
				{ 
		 			flag = true;
				}
		 	}
		}
		if (!flag)
		{
			p.x = 255;
			p.y = 255;
			p.z = 255;
		}
		else
		{
			p.x = 0;
			p.y = 0;
			p.z = 0;	
		}
	});
	erode(dst, dst, Mat(), Point(-1, -1), 4, 1, 1);
	medianBlur(dst, dst, 5);
	return dst;
}
