#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <eklavya4_roboteq/control.h>
#include <std_msgs/Int8.h>
#include <geometry_msgs/Twist.h>
#define radius  0.1 
#define distance  0.64


double linear,angular,vl,vr;
int mode=1;
int estop=0;
eklavya4_roboteq::control manual_control ;
int e=0;
std_msgs::Int8 md;
double linear_v=0, angular_v=0;

void xbee_EStop(const std_msgs::Int8 estop){
e = estop.data;
}

void joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{	  
  
mode=(mode+joy->buttons[4])%2;
estop=(estop+joy->buttons[5])%2;

if(mode==1){



if(estop==1||e==1){
 linear = 0;
 angular = 0;
 manual_control.vl= 0 ;
 manual_control.vr= 0 ;
 
}

manual_control.estop=estop;
 
linear+=(joy->buttons[3]-joy->buttons[0]);
angular+=(joy->buttons[1]-joy->buttons[2]);
manual_control.vl=-(linear*2.5-angular*0.5);
manual_control.vr=-(linear*2.5+angular*0.5);

}
md.data = mode;
manual_control.mode=mode;

}

void autoCallback(const geometry_msgs::Twist auto_command)
{	  
  
if(mode==0){


if(estop==1||e==1){
 linear_v = 0;
 angular_v = 0;
 manual_control.vl= 0 ;
 manual_control.vr= 0 ;
 
}
manual_control.estop= estop;
 
linear_v = auto_command.linear.x;
angular_v = auto_command.angular.z;
manual_control.vl = -((2*linear_v-(angular_v*distance))*15)/(3.14*radius);
manual_control.vr = -((2*linear_v+(angular_v*distance))*15)/(3.14*radius);
 
} 


}




int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_turtle");
  ros::NodeHandle nh_;

  ros::Publisher control, flash;
  ros::Subscriber joy_sub_manual, sub2, auto_sub;
  

  flash = nh_.advertise<std_msgs::Int8>("mode", 1);
  control = nh_.advertise<eklavya4_roboteq::control>("control", 1);
  joy_sub_manual= nh_.subscribe<sensor_msgs::Joy>("joy", 1000, &joyCallback);
  sub2 = nh_.subscribe("xbee_estop", 1000, xbee_EStop);
  auto_sub = nh_.subscribe<geometry_msgs::Twist>("cmd_vel", 1000, &autoCallback);
  

  ros::Rate loop_rate(10);

  
  while (ros::ok())
  {
    
    control.publish(manual_control);
    flash.publish(md);    

    ros::spinOnce();

    loop_rate.sleep();
   
  }


  return 0;
  

  
}
