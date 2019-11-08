/*
 * rosserial PubSub Example
 * Prints "hello world!" and toggles led
 */

#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>

ros::NodeHandle  nh;


void messageCb1( const std_msgs::String& toggle_msg){
  digitalWrite(13, HIGH-digitalRead(13));   // blink the led
}

void messageCb2( const std_msgs::String& toggle_msg){
  digitalWrite(9, HIGH-digitalRead(9));   // blink the led
}

ros::Subscriber<std_msgs::String> sub1("/motor_ctrls/robot1", messageCb1 );
ros::Subscriber<std_msgs::String> sub2("/ctrl_flags/robot1", messageCb2 );

std_msgs::String str_msg;
ros::Publisher chatter("/encoder_data/robot1", &str_msg);

char hello[13] = "hello world!";

void setup()
{
  pinMode(13, OUTPUT);
  pinMode(9, OUTPUT);
  nh.initNode();
  nh.advertise(chatter);
  nh.subscribe(sub1);
  nh.subscribe(sub2);
}

void loop()
{
  str_msg.data = hello;
  chatter.publish( &str_msg );
  nh.spinOnce();
  delay(500);
}
