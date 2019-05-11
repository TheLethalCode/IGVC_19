#include <ros.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Float32.h>

int m=1;

ros::NodeHandle nh;

void flash( const std_msgs::Int8 manual_control){
  m =  manual_control.data ;
}

ros::Subscriber<std_msgs::Int8> sub("mode", &flash );

std_msgs::Int8 estop;

ros::Publisher pub("xbee_estop", &estop);

void setup(){ 
  
  estop.data=0;
  
  pinMode(7, OUTPUT);

  pinMode(10, INPUT);
 
  *digitalPinToPCMSK(10) |= bit (digitalPinToPCMSKbit(10));  // enable pin
    PCIFR  |= bit (digitalPinToPCICRbit(10)); // clear any outstanding interrupt
    PCICR  |= bit (digitalPinToPCICRbit(10)); // enable interrupt for the group
  nh.initNode();
  
  nh.subscribe(sub);

  nh.advertise(pub);
}



void loop(){
   
  if(m){

    if((millis()%1000)<=500){
    digitalWrite(7, HIGH);
    }
    else{
    digitalWrite(7, LOW);  
    }
    
}
  if(1-m){
    
    digitalWrite(7, HIGH);
      
  }

  Serial.println(m); 
  pub.publish(&estop); 
  nh.spinOnce();
  

}
ISR (PCINT0_vect) // handle pin change interrupt for D8 to D13 here
 {    estop.data=1-estop.data;
     
 }
