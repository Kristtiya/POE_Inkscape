/*
 * rosserial Publisher Example
 * Prints "hello world!"
 */

#include <ros.h>
#include <std_msgs/Int8.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *rightMotor = AFMS.getMotor(2); // Right Motor
Adafruit_DCMotor *leftMotor = AFMS.getMotor(1); // Left Motor

ros::NodeHandle  nh;

// initialize variable for the motor speed
int Mspeed = 30;

void messageCb( const std_msgs::Int8& mspeed){
  Mspeed = mspeed->data;
}

std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);
ros::Subscriber<std_msgs::Int8> sub("/motor_speed", &messageCb );

char hello[13] = "hello world!";

void setup()
{
  AFMS.begin();
  nh.initNode();
  nh.advertise(chatter);
}

void loop()
{
  str_msg.data = hello;
  chatter.publish( &str_msg );
  nh.spinOnce();
  setMotorSpeed(Mspeed, Mspeed);
  forwardMotion();
  delay(1000);
}

/// set motor speeds with the parameter
void setMotorSpeed(int RMspeed, int LMspeed) {
  rightMotor->setSpeed(RMspeed+3);
  leftMotor->setSpeed(LMspeed);
}

void forwardMotion() {
  rightMotor->run(FORWARD);
  leftMotor->run(FORWARD);
}

void backwardMotion() {
  rightMotor->run(BACKWARD);
  leftMotor->run(BACKWARD);
}

void turnRight(int t) {
  rightMotor->run(BACKWARD);
  leftMotor->run(FORWARD);
  delay(t);
}

void turnLeft(int t){
  rightMotor->run(FORWARD);
  leftMotor->run(BACKWARD);
  delay(t);
}
