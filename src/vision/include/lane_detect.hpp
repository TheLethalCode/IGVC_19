#ifndef LANE_DETECT
#define LANE_DETECT


#include <bits/stdc++.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/mat.hpp>
using namespace std;
using namespace cv;
#define grid_width 3
// #define grid_height 50
// #define angle_inc 15
// #define thresh 50
Mat lane_detect(Mat img)
{
	// Mat img = imread(argv[1], 0);
	int thresh = 0;//, grid_height = 0;
	namedWindow( "win" , 0);
	int grid_height =0;
	int angle_inc=15;
	createTrackbar("thresh", "win", &thresh, 100);
	createTrackbar("grid_height", "win", &grid_height, 100);
	createTrackbar("angle_inc", "win", &angle_inc, 100);

	//while(1)
	{	
		Mat img1( img.rows , img.cols ,CV_8UC3, Scalar(0, 0, 0));
		vector <Point> vec;
		vector<vector<Point> > contours_white;
		vector<Vec4i> hierarchy_white;
		findContours(img, contours_white, hierarchy_white, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
		Point centroid[contours_white.size()];
		for(int i = 0; i < contours_white.size(); i++)
		{
			// cout << contours_white.size() << endl;
			for(int j = 0; j < contours_white[i].size(); j++)
			{
				centroid[i].x += contours_white[i][j].x;
				centroid[i].y += contours_white[i][j].x;
			}
			centroid[i].x /= contours_white[i].size();
			centroid[i].y /= contours_white[i].size();
			
			// double perimeter=arcLength(contours_white[i],true);
			// double area = contourArea(contours_white[i], false );	
			vec.push_back(Point(centroid[i].y, centroid[i].x));
		}
		//vector<Rect> box_dark(contours_dark.size());
		// for(int i = 0; i < img.rows; i++)
		// {
		// 	for(int j = 0; j < img.cols; j++)
		// 	{
		// 		if(img.at<uchar>(i ,j) > 200)
		// 		{
		// 			vec.push_back(Point (j,i) );
		// 		}
		// 	}
			
		// }
		bool islane[ vec.size()];
		for(int iter=0; iter<vec.size(); iter++)
		{
			// if(arcLength(contours_white[iter],true)/2 > 60)
			// 	int grid_height =	(int) arcLength(contours_white[iter],true)/2;
			// else
			// 	int grid_height = 30;
			// cout << (int) arcLength(contours_white[iter],true)/2 << endl;//sqrt(contours_white[iter].size());
			cout << "a\n";
			int arr_size = 180/angle_inc;
			int arr[arr_size];
			// int arr[arr_size] = {0};
			for(int i = 0; i < arr_size; i++)
			{
				arr[i] = 0;
			}
			int z=0;
			for(int angle = 0;angle < 180; angle+= angle_inc)
			{
				float angle_rad = angle*CV_PI/180;
				for(int i = -(grid_height/2);i < (grid_height/2) + 1; i++)
				{
					for(int j = -( grid_width/2 );j < (grid_width/2) + 1; j++)
					{
						
						int y=vec[iter].y + i*sin( angle_rad ) + j*cos( angle_rad );
						int x=vec[iter].x - i*cos( angle_rad ) + j*sin( angle_rad );
						if(x < img.cols && x>=0 && y < img.rows && y>=0)
						{
							if(img.at<uchar>(y,x)==255)
							{
								arr[z]++;
							}
						}
					}
				}
				z++;
			}
			cout << "b\n";
			// int arr_size=180/angle_inc;
			thresh = grid_height/3;
			sort( arr, arr+arr_size);
			if( arr[arr_size - 1] < thresh )
			{
				islane[iter] = false;
				continue;
			}
			int count = 0;
			for(int angle = 0; angle < arr_size ; angle++)
			{
				if(arr[angle] > thresh)
				{
					count++;
				}
			}
			if( count > 1)
			{
				islane[iter] = false;
				continue;
			}
			islane[iter] = true;

		}
		for(int i = 0; i < vec.size(); i++)
		{
			if( islane[i] )
			{
				// if(vec[i].y >= 0 && vec[i].y < img.rows && vec[i].x >= 0 && vec[i].y < img.cols)
					//img1.at<uchar>(vec[i].y, vec[i].x) = 255;
				cout << "c\n";
				drawContours(img1, contours_white, i, Scalar(255, 255, 255), -1, 8, hierarchy_white);
				cout << "d\n";
			}
		}
		namedWindow("win", 0);
		imshow("win", img1);
		waitKey(1);
	}
	
	//waitKey(0);
	return img;

}

#endif