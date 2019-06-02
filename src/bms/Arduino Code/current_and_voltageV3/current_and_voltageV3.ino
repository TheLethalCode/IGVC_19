//Uses of this node: 1. Flashlight 2. Collect Voltage and current for a battery.
//We are measuring voltage for one 12V battery connected in the 24V rail.
//The SOC of both connected in the motor rail is assumed to be the same.

#define ofset 510
#define Apin A0
#define Vpin A1   //In the circuit there is space for A2 as well, don't connect to it in any condition
#define led 7
//Rate of data collection
#define delaySensor 100
#define delayFilter 500
//Filtering ke liye
#define multFactorV 4.0
#define multFactorA 4.0
#define fixedGainV 0.01
#define fixedGainA 0.007

//ROS ka part
#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int8.h>

//V aur A ke liye
float VperA = 66.0;  //185mV for 5A, 100mV per 20A, 66 for 30A
int rawA,rawV;  
long int t1 = 0, t2 = 0;    //for Loop using millis
float  A = 0, V = 0, Aout=0,Vout=0,gainV,gainA;

ros::NodeHandle n;
std_msgs::Float32 v;
std_msgs::Float32 a;
ros::Publisher pubv("v", &v);   //data is a object of class va
ros::Publisher puba("a",&a);

int m = 1;  
void flash(const std_msgs::Int8& manualOrAutomatic)
{
  m = manualOrAutomatic.data;   //data in std_msgs stored in variable data
}
ros::Subscriber<std_msgs::Int8> sub("mode",&flash); //flash is a Callback function to get data from topic "mode"



void setup() {
 
 Serial.begin(9600);
 
  n.initNode();
  n.advertise(pubv);
  n.advertise(puba);
// n.subscribe(sub);
 pinMode(led,OUTPUT);
 
 delay(100);  //Arduino ko stabilize hone ke liye time
 Aout = abs(float(analogRead(Apin)-ofset))/1024.0*5000/VperA;
 Vout = float(analogRead(Vpin))/1024.0*5.0*13.3/3.3;    //Will not have to change this, it is just scaling according to potential divider

}

void loop() {
  //manaul ya auto ke liye
  if(m)
  {
    if(millis()%1000<500)
    {
      digitalWrite(led,HIGH);
    }
    else
    {
      digitalWrite(led,LOW);
    }
  }

  else
  {
    digitalWrite(led,LOW);
  }
  
 
  
  
  //Data collection ke vaste
  
  if(millis()>= t1 + delaySensor)
  {   
     t1 = millis(); //To get sensor reading every 50ms t1 was put over here. It may have to be put at the end too depending upont the time take nto the other stuff
     rawA = analogRead(Apin);
     rawV = analogRead(Vpin);
      
     
     V += float(rawV)/1024.0*5.0*3.2/1; //I intend to take average value of about 5 values So t2 loop must begin at 250ms
     A += abs(rawA-ofset)/1024.0*5000/VperA ;

     if(millis()>=t2 + delayFilter)
     {  t2 = millis();
        
        V=V/5.0;
        A=A/5.0;

       
        
        //Gain calculation
        if((V+Vout)>0.1)
        {
          gainV = abs(V - Vout)/abs(V + Vout)*multFactorV + fixedGainV;
        }
        else 
          gainV = fixedGainV;
        
        if( (A+ Aout)>0.1)
        {
          gainA = abs(A - Aout)/abs(A + Aout)*multFactorA + fixedGainA;
          
        }
        else 
          gainA = fixedGainA;
       
       
       if(gainA>1)
       {
        gainA=0.99;
       }
       if(gainV>1)
       {
        gainV=0.99;
       }
       
       //Vout calculation
        if(V<0.5)
        {
          Vout =0 ;
        }
        else
        {
          Vout = Vout + abs(gainV)*(V - Vout);
          Vout = abs(Vout);
        }
        
        if(A<0.1)
        {
          Aout = 0;
        }
        else
        {
           Aout = Aout + abs(gainA)*(A - Aout) ;
           Aout = abs(Aout);
        }

        v.data=Vout;
        a.data=Aout;
     
        pubv.publish(&v);
        puba.publish(&a);
        n.spinOnce();
      

        
        V = 0;
        A = 0;
     }
     
     
    
  }
  
}
