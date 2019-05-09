#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <math.h>
#include "ransac.hpp"

using namespace std;
using namespace cv;

#define stepsize 200
#define botlength 90
#define botwidth 30
#define pi 3.14
//Data from RANSAC
int k;//the number of lanes detected by ransac
float a1,a2,a3,b1,b2,b3;//the coefficients of quaderatic polynominals

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

int checklane(int y,int x,Mat img)
{
	img.at<Vec3b>(y,x)={255,255,0};
	if(fabs(a1*y*y+a2*y+a3-x)<30) {
		return 1;
	}
	if(fabs(b1*y*y+b2*y+b3-x)<30) return 2;
	return 0;
}

int isValid_point(Mat img, int i, int j)
{
	float r;

    // if((img.at<Vec3b>(i,j)[0]==255 && img.at<Vec3b>(i,j)[1]==255 && img.at<Vec3b>(i,j)[2]==255) || (img.at<Vec3b>(i,j)[0]==255 && img.at<Vec3b>(i,j)[1]==0 && img.at<Vec3b>(i,j)[2]==0) || (img.at<Vec3b>(i,j)[0]==0 && img.at<Vec3b>(i,j)[1]==0 && img.at<Vec3b>(i,j)[2]==255))
    //     return 0;

    float r1=sqrt(pow(botwidth/2,2)+pow(botlength/2,2));
    int x1,y1;
    for(r=0;r<r1;r++)
    {
    	for(int theta = 0; theta <360;theta++)
    	{
    		x1=j+r*cos((float)theta*pi/180);
    		y1=i-r*sin((float)theta*pi/180);

    		if((img.at<Vec3b>(y1,x1)[0]==255 && img.at<Vec3b>(y1,x1)[1]==255 && img.at<Vec3b>(y1,x1)[2]==255) || (img.at<Vec3b>(y1,x1)[0]==255 && img.at<Vec3b>(y1,x1)[1]==0 && img.at<Vec3b>(y1,x1)[2]==0) || (img.at<Vec3b>(y1,x1)[0]==0 && img.at<Vec3b>(y1,x1)[1]==0 && img.at<Vec3b>(y1,x1)[2]==255))
    			return 0;
    	}
	}
    	return 1;

}

float GetSlope(Mat img,int min,int max)
{
	float min_rad=min*pi/180;
	float max_rad=max*pi/180;
	if(min!=0&&max!=180)
	{
		cout<<"Slope of 1st line :"<<atan(2*a1*(img.rows-stepsize*sin(min_rad))+a2)<<endl;
		cout<<"Slope of 2nd line:"<<atan(2*b1*(img.rows-stepsize*sin(min_rad))+b2)<<endl;
		float d=atan(2*a1*(img.rows-stepsize*sin(min_rad))+a2)+atan(2*b1*(img.rows-stepsize*sin(max_rad))+b2);
		return -d/2;
	}
	else if(min==0&&max!=180)
	{
		cout<<"Slope of 2nd line:"<<atan(2*b1*(img.rows-stepsize*sin(min_rad))+b2)<<endl;
		float d=atan(2*b1*(img.rows-stepsize*sin(max_rad))+b2);
		return -d;
	} 
	else if(min!=0&&max==180)
	{
		cout<<"Slope of 1st line :"<<atan(2*a1*(img.rows-stepsize*sin(min_rad))+a2)<<endl;
		float d=atan(2*a1*(img.rows-stepsize*sin(min_rad))+a2);
		return -d;
	}
	else return 0; 
}

void CheckCircle (Mat img,int *min,int *max)
{
	
	int theta,theta_min=0,theta_max=180,flag_left=0,flag_right=0;
	float theta_rad;	
	for(theta=0;theta<180;theta++)
	{
		theta_rad=theta*pi/180;

		if(checklane(img.rows-stepsize*sin(theta_rad),img.cols/2-stepsize*cos(theta_rad),img)==1)
		{
			if(flag_left==0)
			{
				theta_min=theta;
				flag_left++;
			}

			if(flag_left==1)
			{
				cout<<"theta(check circle):"<<theta<<endl;
				if(abs(theta-theta_min)>10)
				{
					cout<<"ya"<<endl;
					theta_min=theta;
					flag_left++;
				}
			}
		}		

		if(checklane(img.rows-stepsize*sin(theta_rad),img.cols/2-stepsize*cos(theta_rad),img)==2)
		{
			if(flag_right==0)
			{
				theta_max=theta;
				flag_right++;
			}
		}


	}

	*min=theta_min;
	*max=theta_max;
}

/*int checkdim(Mat img,int x, int y)
{
	int i,j;
	i=x-botlength/2;
	for(j=y-botwidth/2;j<y+botwidth/2;j++)
		if((img.at<Vec3b>(i,j)[0]==255 && img.at<Vec3b>(i,j)[1]==255 && img.at<Vec3b>(i,j)[2]==255) || (img.at<Vec3b>(i,j)[0]==255 && img.at<Vec3b>(i,j)[1]==0 && img.at<Vec3b>(i,j)[2]==0) || (img.at<Vec3b>(i,j)[0]==0 && img.at<Vec3b>(i,j)[1]==0 && img.at<Vec3b>(i,j)[2]==255))
			return 0;
	i=x+botlength/2;
	for(j=y-botwidth/2;j<y+botwidth/2;j++)
		if((img.at<Vec3b>(i,j)[0]==255 && img.at<Vec3b>(i,j)[1]==255 && img.at<Vec3b>(i,j)[2]==255) || (img.at<Vec3b>(i,j)[0]==255 && img.at<Vec3b>(i,j)[1]==0 && img.at<Vec3b>(i,j)[2]==0) || (img.at<Vec3b>(i,j)[0]==0 && img.at<Vec3b>(i,j)[1]==0 && img.at<Vec3b>(i,j)[2]==255))
			return 0;
	j=y-botwidth/2;
	for(i=x-botlength/2;i<x+botlength/2;i++)
		if((img.at<Vec3b>(i,j)[0]==255 && img.at<Vec3b>(i,j)[1]==255 && img.at<Vec3b>(i,j)[2]==255) || (img.at<Vec3b>(i,j)[0]==255 && img.at<Vec3b>(i,j)[1]==0 && img.at<Vec3b>(i,j)[2]==0) || (img.at<Vec3b>(i,j)[0]==0 && img.at<Vec3b>(i,j)[1]==0 && img.at<Vec3b>(i,j)[2]==255))
			return 0;
	j=y+botwidth/2;
	for(i=x-botlength/2;i<x+botlength/2;i++)
		if((img.at<Vec3b>(i,j)[0]==255 && img.at<Vec3b>(i,j)[1]==255 && img.at<Vec3b>(i,j)[2]==255) || (img.at<Vec3b>(i,j)[0]==255 && img.at<Vec3b>(i,j)[1]==0 && img.at<Vec3b>(i,j)[2]==0) || (img.at<Vec3b>(i,j)[0]==0 && img.at<Vec3b>(i,j)[1]==0 && img.at<Vec3b>(i,j)[2]==255))
			return 0;
	return 1;
}*/

int WayPts(Mat img,int *theta_min,int *theta_max)
{
	int i,j;
	int theta;
	float theta_head=90;
	CheckCircle(img,theta_min,theta_max);

	cout<<"min:"<<*theta_min<<" max:"<<*theta_max<<endl;

	float d=GetSlope(img,*theta_min,*theta_max);
	d=d*180/pi;

	cout<<"Final Slope:"<<d<<endl;

	int theta_mid=(*theta_min+*theta_max)/2;

	for(theta=0;theta<(*theta_max-*theta_min)/2;theta++)
	{
		// cout<<"theta:"<<theta<<endl;

		i=img.rows-stepsize*sin((theta_mid+theta)*pi/180);
		j=img.cols/2-stepsize*cos((theta_mid+theta)*pi/180);
		if(img.at<Vec3b>(i,j)[0]==255&&img.at<Vec3b>(i,j)[1]==255&&img.at<Vec3b>(i,j)[2]==0)
		{
			// cout<<"theta(+ve):"<<theta<<endl;

			if(isValid_point(img,i,j))
			{
				theta_head=theta_mid+theta;
				break;
			}
		}
		i=img.rows-stepsize*sin((theta_mid-theta)*pi/180);
		j=img.cols/2-stepsize*cos((theta_mid-theta)*pi/180);
		if(img.at<Vec3b>(i,j)[0]==255&&img.at<Vec3b>(i,j)[1]==255&&img.at<Vec3b>(i,j)[2]==0)
		{
			// cout<<"theta(-ve):"<<theta<<endl;

			if(isValid_point(img,i,j))
			{
				theta_head=theta_mid-theta;
				break;
			}
		}

	}

	cout<<"head: "<<theta_head<<endl;
	
	
	return theta_head;

}

int main()
{
	int i,j;
	int theta_min,theta_max;
	Mat img1=imread("/home/naman/untitled(1).png",0);//image with only lanes in black background
	// namedWindow("a",0);
	// imshow("a",img1);
	// waitKey(0);
	// img1=perspective_transform(img1);
	// Mat img(img1.rows,img1.cols,CV_8UC3,Scalar(0,0,0));
	Mat img=imread("/home/naman/untitled(2).png",1);//black image with white obstacles 
	
	model xx=getRansacModel(img1);
	a1=xx.a1;
	a2=xx.b1;
	a3=xx.c1;
	b1=xx.a2;
	b2=xx.b2;
	b3=xx.c2;
	for(i=0;i<img.rows;i++)
	{
		for(j=0;j<img.cols;j++)
		{
			if(fabs(a1*i*i+a2*i+a3-j)<3) 
				{
					img.at<Vec3b>(i,j)[0]=255;
					img.at<Vec3b>(i,j)[1]=0;
					img.at<Vec3b>(i,j)[2]=0;
				}
		}
	}
	for(i=0;i<img.rows;i++)
	{
		for(j=0;j<img.cols;j++)
		{
			if(fabs(b1*i*i+b2*i+b3-j)<3)
			{
				img.at<Vec3b>(i,j)[0]=0;
				img.at<Vec3b>(i,j)[1]=0;
				img.at<Vec3b>(i,j)[2]=255;
			}
		}
	}
	cout<<img.rows<<" "<<img.cols<<endl;
	float ans=(WayPts(img,&theta_min,&theta_max))*pi/180;
	cout<<theta_min<<" "<<theta_max<<endl;
	float slope=GetSlope(img,theta_min,theta_max);

	RotatedRect rRect= RotatedRect(Point2f(img.cols/2-stepsize*cos(ans),img.rows-stepsize*sin(ans)),Size2f(botwidth,botlength),slope*180/pi);
	Point2f vertices_float[4];
	rRect.points(vertices_float);
	Point vertices[4];
	vertices[0]=vertices_float[0];
	vertices[1]=vertices_float[1];
	vertices[2]=vertices_float[2];
	vertices[3]=vertices_float[3];
	fillConvexPoly(img,vertices,4,Scalar(0,255,0));

	// for(i=-botlength/2;i<botlength/2;i++)
	// 	for(j=-botwidth/2;j<botwidth/2;j++)
	// 		img.at<Vec3b>(img.rows-stepsize*sin(ans)+i,img.cols/2-stepsize*cos(ans)+j)[1]=255;

	namedWindow("win",0);
	namedWindow("win1",0);

		
	imshow("win",img);
	imshow("win1",img1);
	
	waitKey(0);
	cout<<ans<<endl;

	return 0;
}
