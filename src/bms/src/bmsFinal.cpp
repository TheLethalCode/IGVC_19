//Gets V and A from topic /v /a respectively, publishes Soc to topic /SOC 

#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Float32.h>
#include <math.h>
#include <iostream>

const float R=0.23;	//Internal resistance of the battery, may have to calculate it

const float a=12.84	;//The 6 battery parameters
const float	b=-0.62;
const float	m =0.65;
const float	c =-0.2;
const float	d =0.6;
const float	n =0.6;

const float epsilon =1.0e-4;    //Float compare karne ke liye aise hi
float V=12,A=3,OCV=13.2,z=0.99;	//z is battery left

using namespace ros;
using namespace std;

float f(float z)      //OCV = F(SOC), f = F(SOC) - OCV
{ 
	float temp = a + b*pow((abs(log(abs(z)))),m)+ c*z + d*exp(n*(z-1)) - OCV; 
	return temp;
}

float fd(float z)     //Upar vale ka derivative
{ 	float temp = b*m*pow((abs (log(abs(z))) ),m)/( z*log(abs(z)) ) + c + d*n*exp(n*(z-1));	
	   return temp;
}

  void vCallBack(const std_msgs::Float32& v)
  {
    V=v.data;
   // ROS_INFO("vCallBack called\n");
  }
  void aCallBack(const std_msgs::Float32& a)
  {
    A=a.data;
   // ROS_INFO("aCallBack called\n");
  }

float calc()      //Newton rapson se f ka root nikal rahe hain: OCV = F(SOC), f = F(SOC)-OCV ka root nikal rahe hain for every value of V published by arduino
{	float e = 1,zprev=z;
  OCV=V+A*R;    
  ROS_INFO("%f %f %f\n",OCV,A,V);
  
  if(OCV>13.2)
  {
    return 1.0;
  }

  while(abs(e)>epsilon)
	{
		z = zprev -(f(zprev)/fd(zprev));
		e = z-zprev;
		zprev = z;
					
  }
	
	return abs(z);
}




int main(int argc, char **argv)
{
 
  ros::init(argc, argv, "BMS");		//BMS is the name of the node

  ros::NodeHandle nh;

  ros::Publisher soc;			    //Publisher is a pointer/variable.
                          		//nodefandle.advertise returns a publisher and subscribe returns a subscriber 
  ros::Subscriber Vol;
  ros::Subscriber Amp;
  soc = nh.advertise<std_msgs::Int8>("SOC",1); //Topicname, buffer size,(callback in case of a subscriber)
  Vol = nh.subscribe("v",1,vCallBack);
  Amp = nh.subscribe("a",1,aCallBack);
  ros::Rate loop_rate(2);

  while (ros::ok())
  {
    std_msgs::Int8 msg;
    z = calc();
    msg.data = int(z*100);
    soc.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  
  }


  return 0;
}
