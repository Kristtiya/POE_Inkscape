#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int32MultiArray.h>

ros::NodeHandle  nh;

// initialize variable for the motor speed
int speeds[2] = {0,0};
long enc_data[3] = {0,0,0};

void ctrlsCb( const geometry_msgs::Twist& mspeed){
  speeds[0] = mspeed.linear.x;
  speeds[1] = mspeed.angular.z;
}

ros::Subscriber<geometry_msgs::Twist> ctrlSub("/motor_ctrls/robot1", &ctrlsCb );
std_msgs::Int32MultiArray recv_msg;
ros::Publisher encoder_pub("/encoder_data/robot1", &recv_msg);

void setup()
{
  nh.initNode();
  nh.advertise(encoder_pub);
  nh.subscribe(ctrlSub);
}

void loop()
{
  enc_data[0] = 0;
  enc_data[1] = 0;
  enc_data[2] = millis();
  recv_msg.data = enc_data;
  recv_msg.data_length = 3;
  encoder_pub.publish( &recv_msg );
  nh.spinOnce();
  delay(50);
}
