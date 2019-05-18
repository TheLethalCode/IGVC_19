#include <iostream>
#include <stdio.h>
#include <string.h>
#include "geometry_msgs/Twist.h"

#include <ros/ros.h>
#include "eklavya4_roboteq/SetSpeed.h"

#include "eklavya4_roboteq/RoboteqDevice.h"
#include "eklavya4_roboteq/ErrorCodes.h"
#include "eklavya4_roboteq/Constants.h"
#include "eklavya4_roboteq/diagnose_msg.h"
#include "eklavya4_roboteq/RoboteqDevice.cpp"
using namespace std;
#define r 0.11   //the radius of wheel in centimetre

int main(int argc, char **argv)
{   
    ros::init(argc, argv, "current");
eklavya4_roboteq::diagnose_msg diagnose_struct;
//ros::Publisher controller_pub;
ros::Publisher diagnose;
RoboteqDevice device;
    ros::init(argc, argv, "motor_controller_server");
    ros::NodeHandle n;    
diagnose = n.advertise<eklavya4_roboteq::diagnose_msg>("diagnosis",1000);	
	
	ros::Rate loop_rate(10);
	int status=0;
	int c1=0,c2=0,v=0;
	status = device.Connect("/dev/serial/by-id/usb-Roboteq_Motor_Controller_498954A73235-if00");
	ROS_INFO("Connection status = %f", status );
	//eklavya4_roboteq::diagnose_msg diagnose_struct, a;
	while(ros::ok())
	{
	status=device.GetValue(_BATAMPS,1,c1);
        ROS_INFO("Current Read status = %f", status );
	status=device.GetValue(_BATAMPS,2,c2);
	status=device.GetValue(_VOLTS,2,v);

	diagnose_struct.M1_current=c1;
	diagnose_struct.M2_current=c2;
	diagnose_struct.Voltage=v;

	diagnose.publish(diagnose_struct);
	
     
	ros::spinOnce();
	loop_rate.sleep();
	}
    //ros::spin();
	
	//device.Disconnect();
  
    return 0;
}
