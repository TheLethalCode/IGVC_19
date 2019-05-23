#include"opencv2/highgui/highgui.hpp"
#include"opencv2/imgproc/imgproc.hpp"
#include"opencv2/core/core.hpp"
#include<iostream>
#include <eigen3/Eigen/Dense>
#include <eigen3/unsupported/Eigen/MatrixFunctions>
#include<math.h>
#include"ransac.hpp"

using namespace std;
using namespace cv;
using namespace Eigen;


#define ppm 100

Matrix<float,6,1> g(Matrix<float,6,1> x,float theta1,float theta2,float A, float B)
{
	Matrix<float,6,1> result;
	float P1=cos(theta2-theta1)-2*x(1,0)*B*sin(theta2-theta1)-x(1,0)*sin(theta2-theta1);
	float P2=cos(theta2-theta1)-2*x(4,0)*B*sin(theta2-theta1)-x(4,0)*sin(theta2-theta1);
	result<< x(0,0) * pow( cos( theta2 - theta1 ) , 2 ) / P1,
			( sin( theta2 - theta1 ) + 2 * x(0,0) * B * cos( theta2 - theta1 ) + x(1,0) * cos( theta2 - theta1 )) / P1, 
			( -A + x(0,0) * B * B + x(1,0) * B + B ) / P1,
			x(3,0) * pow( cos( theta2 - theta1 ) , 2 ) / P2,
			( sin( theta2 - theta1 ) + 2 * x(3,0) * B * cos( theta2 - theta1 ) + x(4,0) * cos( theta2 - theta1 )) / P2, 
			( -A + x(3,0) * B * B + x(4,0) * B + B ) / P2;
	return result;
}

Matrix<float,6,6> ukf_lanes(Parabola &prev,Parabola now,float x1,float x2,float y1,float y2,float theta1,float theta2,Matrix<float,6,6> P)
{
	float A=(x2-x1)*cos(theta1) - (y2-y1)*sin(theta1);
	float B=(x2-x1)*(sin(theta1)) + (y2-y1)*cos(theta1);
	Matrix<float,6,1> xhat,u_new;
	u_new<<0,
			0,
			0,
			0,
			0,
			0;
	Matrix<float,6,1> chi[13];
	xhat<<prev.a1,
			prev.b1,
			prev.c1,
			prev.a2,
			prev.b2,
			prev.c2;
	cout<<"Previous input"<<endl<<xhat<<endl<<endl;
	// char c;
	// cin>>c;
	Matrix<float,6,1> Y;
	Y<<now.a1,
		now.b1,
		now.c1,
		now.a2,
		now.b2,
		now.c2;
	cout<<"old ransac"<<endl<<Y<<endl<<endl;
	// cin>>c;
	Matrix<float,6,6> P_new;
	P_new<<0,0,0,0,0,0,			//defined in main code
		0,0,0,0,0,0,
		0,0,0,0,0,0,
		0,0,0,0,0,0,
		0,0,0,0,0,0,
		0,0,0,0,0,0;
	cout<<"Previous Corvarience"<<endl<<P<<endl<<endl;
	// cin>>c;
	Matrix<float,6,6> temp=P.sqrt();
	for(int i=0;i<6;i++)
		for(int j=0;j<6;j++)
			if(temp(i,j)<0)
				temp(i,j)=-temp(i,j);
	chi[0]=xhat;
	for(int i=1;i<=6;i++)
	{
		Matrix<float,6,1> sig;
		int j=0;
		sig<<temp(j++,i-1),
			temp(j++,i-1),
			temp(j++,i-1),
			temp(j++,i-1),
			temp(j++,i-1),
			temp(j++,i-1);
		chi[i] = xhat + sqrt(24)*sig; 
	}
	for(int i=7;i<=12;i++)
	{
		Matrix<float,6,1> sig;
		int j=0;
		sig<<temp(j++,i-7),
			temp(j++,i-7),
			temp(j++,i-7),
			temp(j++,i-7),
			temp(j++,i-7),
			temp(j++,i-7);
		chi[i] = xhat - sqrt(24)*sig; 
	}
	/*for(int i=0;i<13;i++)
	{
		cout<<"chi "<<i<<endl<<chi[i]<<endl<<endl;
		cin>>c;
	}*/
	float w[13];
	w[0] = 0.75;
	for(int i = 1; i<13 ; i++)
		w[i] = 1/48.00;
	for(int i=0;i<=12;i++)
	{
		u_new += w[i]*g(chi[i],theta1,theta2,A,B);
	}
	cout<<"u_new "<<endl<<u_new<<endl;
		// cin>>c;
	for(int i=0;i<=12;i++)
	{
		P_new += w[i]*(g(chi[i],theta1,theta2,A,B) - u_new)*(g(chi[i],theta1,theta2,A,B) - u_new).transpose();
	}
	cout<<"P_new "<<endl<<P_new<<endl;
		// cin>>c;
	Matrix<float,6,6> R;
	R<<pow(now.a1-prev.a1,2),0,0,0,0,0,
		0,pow(now.b1-prev.b1,2),0,0,0,0,
		0,0,pow(now.c1-prev.c1,2),0,0,0,
		0,0,0,pow(now.a2-prev.a2,2),0,0,
		0,0,0,0,pow(now.b2-prev.b2,2),0,
		0,0,0,0,0,pow(now.c2-prev.c2,2);
	//cout<<"R"<<R<<endl;
	Matrix<float,6,6> I;
	I<<1,0,0,0,0,0,
		0,1,0,0,0,0,
		0,0,1,0,0,0,
		0,0,0,1,0,0,
		0,0,0,0,1,0,
		0,0,0,0,0,1;
	Matrix<float,6,6> H;
	H<<1,0,0,0,0,0,
		0,1,0,0,0,0,
		0,0,1,0,0,0,
		0,0,0,1,0,0,
		0,0,0,0,1,0,
		0,0,0,0,0,1;
	Matrix<float,6,6> K;
	K=P_new*(H*P_new*H.transpose()+R).transpose();
	Matrix<float,6,1> u_final;
	u_final=u_new+K*(Y-u_new);
	P=(I-H*K)*P_new;
	prev.a1=u_final(0,0);
	prev.b1=u_final(1,0);
	prev.c1=u_final(2,0);
	prev.a2=u_final(3,0);
	prev.b2=u_final(4,0);
	prev.c2=u_final(5,0);
	cout<<"final values"<<endl<<u_final<<endl<<endl;
	return P;
}

