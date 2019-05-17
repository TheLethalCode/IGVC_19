#include"opencv2/highgui/highgui.hpp"
	#include"opencv2/imgproc/imgproc.hpp"
	#include"opencv2/core/core.hpp"
	#include<iostream>
	#include <opencv2/opencv.hpp>
	#include <string>

	using namespace std;
	using namespace cv;

	#define alpha 100
	#define pi 3.14159
	#define threshup 10
	#define areathresh 2500
	#define clearance 6
	#define threshtan 100

	#define threshb 0
	#define threshg 0
	#define threshr 0

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

	float mod (Point pt)
	{
		float val = pt.x*pt.x + pt.y*pt.y;
		if (val<0)
		{
			cout<<1;
		}
		return sqrt (val);
	}
	typedef Point3_<uint8_t> Pixel;
	void blurContour(const vector<Point>& src, vector<Point>& dst, float k)
	{
		int siz = src.size();
		int off = (k-1)/2;
		for (int i = 0; i < siz; i++)
		{
			Point sum(0, 0);
			for (int j = 0; j < k; j++)
			{
				sum += src[(i+j)%siz];
				if ((i+j)%siz < 0 || (i+j)%siz > siz)
				{
					cout<<1;
				}
			}
			dst.push_back(sum/k);
		}
		if (dst[0] == dst[siz-1])
		{
			cout<<1;
		}
		//cout<<dst[siz-1];
	}

	struct Operator
	{
		void operator ()(Pixel &p,const int *position) const
		{
		    Pixel img2;
		    bool flag = false;
		 //    (img7.x) = (img2.x) = (p.x);		//blue
			// (img7.y) = (img2.y) = p.y+mapped(p.y, 0, 255, alpha, 0);		//green
			// (img2.z) = max(p.z-mapped(p.z, 0, 255, alpha, 0), 0); 		//red2
			// (img7.z) = p.z;		//red7*/
		    (img2.x) = max(p.x-mapped(p.z, 0, 255, 2*alpha, 0), 0);		//blue
			(img2.y) = p.y+mapped(p.y, 0, 255, alpha, 0);		//green
			(img2.z) = max(p.z-mapped(p.z, 0, 255, alpha, 0), 0); 		//red2
			
			if (img2.y > img2.z && img2.z> img2.x) 
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
			//p = img2;
		}
	};

	Mat &CreateObjectMask(Mat &img) 
	{
		// namedWindow("fulfinal",WINDOW_NORMAL);
		
		img.forEach<Pixel>(Operator());
		// img.forEach<Pixel>([](Pixel &p, const int * position) -> void 
		// {
		//     Pixel img2;
		//     bool flag = false;
		//  //    (img7.x) = (img2.x) = (p.x);		//blue
		// 	// (img7.y) = (img2.y) = p.y+mapped(p.y, 0, 255, alpha, 0);		//green
		// 	// (img2.z) = max(p.z-mapped(p.z, 0, 255, alpha, 0), 0); 		//red2
		// 	// (img7.z) = p.z;		//red7*/
		//     (img2.x) = (p.x);		//blue
		// 	(img2.y) = p.y+mapped(p.y, 0, 255, alpha, 0);		//green
		// 	(img2.z) = max(p.z-mapped(p.z, 0, 255, alpha, 0), 0); 		//red2
			
		// 	if (img2.y > img2.z && img2.z> img2.x)
		// 	{
		// 		if (	
		// 				!((p.x < threshb || p.x > threshbW) && 
		// 				(p.y < threshg || p.y > threshgW) && 
		// 				(p.z < threshr || p.z > threshrW) )
		// 				)
		// 			{ 
		// 	 			flag = true;
		// 			}
		// 	}
			
		// 	if (!flag)
		// 	{
		// 		p.x = 255;
		// 		p.y = 255;
		// 		p.z = 255;
		// 	}
		// 	else
		// 	{
		// 		p.x = 0;
		// 		p.y = 0;
		// 		p.z = 0;	
		// 	}
		// });
		erode(img, img, Mat(), Point(-1, -1), 4, 1, 1);
		medianBlur(img, img, 5);
		return img;
	}
	Mat object_remove(Mat img1)
	{
		float thrushup1 = tan(pi*(90+threshup)/180), threshup2 = tan(pi*(90-threshup)/180), thrushdown1 = tan(pi*(threshup)/180), threshdown2 = tan(pi*(-threshup)/180);
		//namedWindow("fulfinal",WINDOW_NORMAL);
		//namedWindow("red",WINDOW_NORMAL);
		//namedWindow("green",WINDOW_AUTOSIZE);
		//namedWindow("blue",WINDOW_NORMAL);
		//namedWindow("semifinal", WINDOW_NORMAL);

		int num=0;
			Mat img3, grad_x, grad_y, abs_grad_x, abs_grad_y;
			Mat img2(img1.rows, img1.cols, CV_8UC1, Scalar(0));
			Mat img4(img1.rows, img1.cols, CV_8UC3, Scalar(0));
			CreateObjectMask(img1);

			cvtColor(img1, img2, COLOR_RGB2GRAY);
			dilate(img2, img2, Mat(), Point(-1, -1), 1, 1, 1);
			vector<vector<Point> > contours;
			
			vector<Vec4i> hierarchy;
			findContours( img2, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE );

			for(size_t i = 0; i < contours.size(); i++) 
			{
	    		double area = contourArea(contours[i]);
	    		if (area < areathresh)
	    		{
	    			contours.erase(contours.begin()+i);
	    			i--;
	   			//	cout<<i<<endl;
	    		}
			}

			hierarchy.clear();

			int contsize = contours.size();
			vector<vector<Point> > simple(contsize);

			for( int i = 0; i < contsize; i++ )
		    {
		    	blurContour(contours[i], simple[i], 11);
		    	//eraseRednt(simple[i]);
		    }

		 //    for( int i = 0; i < contsize; i++ )
		 //    {
			// 	//scaleContour(simple[i], scaled[i], 5);
			// 	//ContourNormal(simple[i], scaled[i]);
			// }
			
			//cout<<scaled[2];
			Scalar color2( 255, 255, 255);
			for( int i = 0; i < contsize; i++ )
			{
				drawContours( img4, simple, i, color2, -40, 8, vector<Vec4i>(), 0, Point());
				drawContours( img4, simple, i, color2, 10, 8, vector<Vec4i>(), 0, Point());	
		    }
		    //floodFill(img4, Point(500, 500), color2);//Point( mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00 ), color2);
		    //cout<<contours[2];

		    contours.clear();
		    simple.clear();
		    return img4	;
	}