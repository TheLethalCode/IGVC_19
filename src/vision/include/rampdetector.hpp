//This code returns a Mat a on which red color represents lanes whereas blue rectangle is bounding the ramp. 

#ifndef RAMPDETECTOR
#define RAMPDETECTOR


//NOTE threshold for CONTOUR AREA is 25000, for image size 640*480, it must be scaled according to size of image
#define contour_area_lim 25000


#include"opencv2/highgui/highgui.hpp"
#include"opencv2/core/core.hpp"
#include"opencv2/imgproc/imgproc.hpp"
#include<iostream>
using namespace cv;
using namespace std;
vector<vector<Point> > contours;

    
int val(int x)
{
	if(x<0) return 0;
	if(x>255) return 255;
	else return x;
}
Mat rampdetector(Mat a)
{
	
	Mat b2r(a.rows,a.cols,CV_8UC1,Scalar(0));
	Mat bnw(a.rows,a.cols,CV_8UC1,Scalar(0));
	Mat copy,c1,blak(a.rows,a.cols,CV_8UC1,Scalar(0));
	int i,j,k;

		
			Mat out(a.rows,a.cols,CV_8UC3,Scalar(0, 0, 0));
			
			for(i=0;i<a.rows;i++)
			{
				for(j=0;j<a.cols;j++)
				{
					b2r.at<uchar>(i,j)=val(2*a.at<Vec3b>(i,j)[0]-a.at<Vec3b>(i,j)[1]);
				}

			}
			for(i=0;i<a.rows;i++)
			{
				for(j=0;j<a.cols;j++)
				{
					if(b2r.at<uchar>(i,j)>60&&(a.at<Vec3b>(i,j)[0]+a.at<Vec3b>(i,j)[1]+a.at<Vec3b>(i,j)[2])<540)
						bnw.at<uchar>(i,j)=255;
					else
						bnw.at<uchar>(i,j)=0;
				}
			}
			copy=bnw.clone();
			c1=blak.clone();
			findContours(bnw,contours,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE);
			/*cout<<contours.size()<<endl;*/
			vector<Vec4i> hierarchy;
		    vector<Rect> boundRect( contours.size() );
		    vector<vector<Point> > contours_poly( contours.size() );
				for(i=0,j=0;i<contours.size();i++)
				{
					/*cout<<"yes"<<endl;*/	
					approxPolyDP( Mat(contours[i]), contours_poly[i], 3, true );
					/*cout<<"no"<<endl;*/
					boundRect[i] = boundingRect( Mat(contours_poly[i]) );
					if(contourArea(contours[i])>contour_area_lim)
						j=i;
				}

				for(i=boundRect[j].tl().y;i<boundRect[j].br().y;i++)
				{
					for(k=boundRect[j].tl().x;k<boundRect[j].br().x;k++)
					{
						if(bnw.at<uchar>(i,k)==0&&(a.at<Vec3b>(i,k)[0]+a.at<Vec3b>(i,k)[0]+a.at<Vec3b>(i,k)[0])>600)
							out.at<Vec3b>(i,k)={0,0,255};
					}
				}
				if(j!=0)
				{
				drawContours(c1,contours,j,125,1,8);
				rectangle( out, boundRect[j].tl(), boundRect[j].br(), Scalar(255,0,0), 2, 8, 0 );
				}		
				

			/*imshow("W3",c1);
			imshow("W",out);


			imshow("W1",b2r);
			imshow("W2",copy);*/
			/*waitKey(50);*/
			return out;             
}


#endif
