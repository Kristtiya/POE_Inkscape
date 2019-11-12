#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *rightMotor = AFMS.getMotor(2); // Right Motor
Adafruit_DCMotor *leftMotor = AFMS.getMotor(1); // Left Motor

ros::NodeHandle  nh;

// initialize variable for the motor speed
int V = 30;
int w = 1;
int temp = 0;
char info[16] = "";

void ctrlsCb( const geometry_msgs::Twist& mspeed){
  V = mspeed.linear.x;
  w = mspeed.angular.z;
}

void ctrlsCb( const geometry_msgs::Twist& mspeed){
  V = mspeed.linear.x;
  w = mspeed.angular.z;
}

ros::Subscriber<geometry_msgs::Twist> sub("/motor_ctrls/robot1", &ctrlsCb );
ros::Subscriber<geometry_msgs::Twist> sub("/motor_ctrls/robot1", &flagsCb );
geometry_msgs::Twist recv_msg;
ros::Publisher chatter("/recv_m_speed", &recv_msg);

void setup()
{
  AFMS.begin();
  nh.initNode();
  nh.advertise(chatter);
  nh.subscribe(sub);
}

void loop()
{
  recv_msg.linear.x = V;
  recv_msg.angular.z = w;
  chatter.publish( &recv_msg );
  temp = V;
  itoa(temp, info, 10);
  nh.loginfo(info);
  nh.spinOnce();
  delay(50);
}
