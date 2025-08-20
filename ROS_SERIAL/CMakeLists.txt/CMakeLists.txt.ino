#include <ros.h>
#include <geometry_msgs/Twist.h>

double RX=0;
double RZ=0;

ros::NodeHandle nh;

void messageCb( const geometry_msgs::Twist& msg){
  RX= msg.linear.x;
  RZ= msg.linear.z;
  
  if (RX >=2.0){
    adelante();
    
    }
  
  if (RX <=0){
    atras();
    
    }
    
  if (RZ >=2.0){
    derecha();
    
    }
  
  if (RZ <=0){
    izquierda();
    
    }
 }

ros::Subscriber<geometry_msgs::Twist> sub("turtle1/cmd_vel", messageCb ); 

void adelante() {
  pinMode(13, HIGH);
  }
  
  void atras() {
  pinMode(13, LOW);
  }
  
  void derecha() {
  pinMode(12, HIGH);
  }
  
  void izquierda() {
  pinMode(12, LOW);
  }
  
  void setup()
{
  pinMode(13, OUTPUT);
  pinMode(12, OUTPUT);
  nh.initNode();
  nh.subscribe(sub);
}

void loop()
{

  nh.spinOnce();
  
 } 
 
 //ls -l /dev | grep ACM
 //sudo chmod 777 /dev/ttyACM1
 // rosrun rosserial_python  serial_node.py  /dev/ttyACM1
 
 //  roscore
 //  rosrun turtlesim turtle_teleop_key 
 //  rosrun turtlesim turtlesim_node 
 //  rosrun rosserial_python  serial_node.py  /dev/ttyACM1
 //  rostopic echo /turtle1/cmd_vel
