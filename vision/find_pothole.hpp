#ifndef FIND_POTHOLE
#define FIND_POTHOLE

#include <opencv2/opencv.hpp>
#include <iostream>
#include <math.h>

using namespace std;
using namespace cv;

// Mat img=imread("ab.png",1);

Mat perspective_transform(Mat img)
{

	Mat top_view(img.rows,img.cols,CV_8UC1,Scalar(0));

	vector <Point2f> rect(4);
	vector <Point2f> lane1(4);
	//vector <Point2f> lane2(3);

	//points used to find homography matrix
	rect[3].x=440;
	rect[3].y=465;
	rect[2].x=440;
	rect[2].y=345;
	rect[1].x=717;
	rect[1].y=345;
	rect[0].x=717;
	rect[0].y=465;

	lane1[3].x=77;
	lane1[3].y=550-240;
	lane1[2].x=159;
	lane1[2].y=431-240;
	/*lane1[2].x=229;
	lane1[2].y=275;*/

	lane1[0].x=1132;
	lane1[0].y=510-240;
	lane1[1].x=1037;
	lane1[1].y=400-240;
	/*lane1[5].x=900;
	lane1[5].y=254;*/

	Mat h=findHomography(lane1,rect);

	// cout<<h<<endl;

	warpPerspective(img,top_view,h,top_view.size());

	return top_view;

}


Mat grass_rm(Mat initial)
{
	Mat temp(initial.rows,initial.cols,CV_8UC1,Scalar(0));
	int i,j;
	//filter=temp.clone();

	for (i = 0; i < initial.rows; ++i)
	{
		for (j=0; j < initial.cols; ++j)
		{
			int pixel=2*initial.at<Vec3b>(i,j)[0]-initial.at<Vec3b>(i,j)[1];
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
	return temp;
}

Mat draw_pothole_from_cannied_img(Mat cannied_img)
{
	int i;

	vector<vector<Point> > contours;
 	 vector<Vec4i> hierarchy;
     findContours( cannied_img, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

     //Area of contour
     float area,perimeter;
     int mark=-1;

     //finding pothole among contours
     for(i=0;i<contours.size();i++)
     {
     	area=contourArea(contours[i]);
     	perimeter=arcLength(contours[i],true);

     	if(area>140){
     		// cout<<(perimeter/sqrt(area))<<endl;
     		if((perimeter/sqrt(area))<4 && (perimeter/sqrt(area))>3){
     			mark=i;
     			cout<<mark<<endl;
     		}
     	}
     }

     // cout<<mark<<endl;

     //initialize image for output
     Mat drawing = Mat::zeros( cannied_img.size(), CV_8UC3 );

     //if pothole is detected then only draw pothole
     if(mark!=-1){

       Scalar color = Scalar(0,255,255);
       drawContours( drawing, contours, mark, color, 2, 8, hierarchy, 0, Point() );
     }

       return drawing;
}

Mat find_pothole(Mat img)
{
	int i,j;
	Mat gray(img.rows,img.cols,CV_8UC1,Scalar(0));//grass removed image
	
	//top view of original image
	Mat top_view(img.rows,img.cols,CV_8UC1,Scalar(0));

	//image on which pothole is drawn
	Mat drawing(img.rows,img.cols,CV_8UC1,Scalar(0));

	top_view=perspective_transform(img);
	GaussianBlur(top_view,top_view,Size (5,5),0,0,BORDER_DEFAULT);
	gray=grass_rm(top_view);
	Canny(gray,gray,200,400,3);
	drawing=draw_pothole_from_cannied_img(gray);

	return drawing;
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