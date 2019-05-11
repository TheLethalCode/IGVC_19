#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <eklavya4_roboteq/control.h>
#define radius
#include <std_msgs/Int8.h>
#define distance
#

class TeleopTurtle
{
public:
  TeleopTurtle();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

  ros::NodeHandle nh_;

  ros::Publisher control, flash;
  ros::Subscriber joy_sub_manual,joy_sub_auto;

};

double linear,angular,vl,vr;
int mode=1;
int estop=0;

TeleopTurtle::TeleopTurtle()
{
  flash = nh_.advertise<std_msgs::Int8>("mode", 1);
  control = nh_.advertise<eklavya4_roboteq::control>("control", 1);
  joy_sub_auto = nh_.subscribe<sensor_msgs::Joy>("cmd_vel", 10, &TeleopTurtle::joyCallback, this);
  joy_sub_manual= nh_.subscribe<sensor_msgs::Joy>("joy", 10, &TeleopTurtle::joyCallback, this);

}

void TeleopTurtle::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{	
  eklavya4_roboteq::control manual_control;
  std_msgs::Int8 md;
  //eklavya4_roboteq::control autonomous_control;
  mode=(mode+joy->buttons[0])%2;
if(mode==1){
estop=(estop+joy->buttons[5])%2;
manual_control.estop=estop;
manual_control.mode=mode; 
linear=joy->axes[1];
angular=joy->axes[2];
manual_control.vl=(0.5*linear-angular)*250;
manual_control.vr=(0.5*linear+angular)*250;
md.data = mode;
}
control.publish(manual_control);
flash.publish(md);	
}

/*void TeleopTurtle::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{	
  
if(mode==0){ 
linear=joy->axes[1];
angular=joy->axes[2];
vl=(0.5*linear-angular)*350;
vr=(0.5*linear+angular)*350;
control.publish(manual_control);
}	
}

*/
int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_turtle");
  TeleopTurtle teleop_turtle;

  ros::spin();
}
