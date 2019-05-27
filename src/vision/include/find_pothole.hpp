#ifndef FIND_POTHOLE
#define FIND_POTHOLE

#include <opencv2/opencv.hpp>
#include <bits/stdc++.h>
#include <math.h>

using namespace std;
using namespace cv;

// Mat img=imread("ab.png",1);

Mat draw_pothole_from_cannied_img(Mat cannied_img,Mat img)
{
	int i;

	vector<vector<Point> > contours;
 	 vector<Vec4i> hierarchy;
     findContours( cannied_img, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

     //Area of contour
     float area,perimeter;
     vector <int > mark;

     //finding pothole among contours
     for(i=0;i<contours.size();i++)
     {
     	area=contourArea(contours[i]);
     	perimeter=arcLength(contours[i],true);

     	if(area>140){
     		// cout<<(perimeter/sqrt(area))<<endl;
     		if((perimeter/sqrt(area))<4 && (perimeter/sqrt(area))>3){
     			mark.push_back(i);
     			cout<<mark[i]<<endl;
     		}
     	}
     }

     // cout<<mark<<endl;

     //initialize image for output
     // Mat drawing = Mat::zeros( cannied_img.size(), CV_8UC1 );

     //if pothole is detected then only draw pothole
     for(i=0;i<mark.size();i++){

       Scalar color = Scalar(255);
       //drawContours( drawing, contours, mark, color, 2, 8, hierarchy, 0, Point() );
       // drawContours(drawing,contours,mark[i],Scalar(255),CV_FILLED);
       drawContours(img,contours,mark[i],Scalar(0),CV_FILLED);
       cout<<"pothole found"<<endl;
     }

       return img;
}

Mat find_pothole(Mat img,Mat costmap)
{
	int i,j;
	Mat gray = img.clone();

	//image on which pothole is drawn
	Mat drawing(img.rows,img.cols,CV_8UC1,Scalar(0));

	// top_view=perspective_transform(img);
	GaussianBlur(gray,gray,Size (5,5),0,0,BORDER_DEFAULT);
	// gray=grass_rm(top_view);
	Canny(gray,gray,200,400,3);

	costmap=draw_pothole_from_cannied_img(gray,costmap);

	return costmap;
}

/*int main()
{

	VideoCapture vid("/home/naman/Potholes/0210-0299.mp4");
	Mat img;//image to be worked upon
	Mat drawing(img.rows,img.cols,CV_8UC1,Scalar(0));
	
	while(1)
	{

		vid>>img;
		// img=imread("ab.png",1);
		drawing=find_pothole(img);

    	namedWindow( "Contours", CV_WINDOW_AUTOSIZE );
		namedWindow("a",0);
		imshow( "Contours", drawing );
		imshow("a",img);
		waitKey(900);
}

	return 0;
}*/

#endif