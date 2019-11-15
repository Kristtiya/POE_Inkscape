#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "motor_speed_setter");
  ros::NodeHandle n;

  ros::Publisher chatter_pub = n.advertise<geometry_msgs::Twist>("/motor_speed", 5);

	int m_speed = 50;

  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    geometry_msgs::Twist msg;
    msg.linear.x = m_speed;
		msg.angular.z = 5;
    chatter_pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
