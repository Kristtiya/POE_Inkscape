#include <ros.h>
#include <std_msgs/Int8.h>
#include <std_msgs/String.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *rightMotor = AFMS.getMotor(2); // Right Motor
Adafruit_DCMotor *leftMotor = AFMS.getMotor(1); // Left Motor

ros::NodeHandle  nh;

// initialize variable for the motor speed
int Mspeed = 30;

void messageCb( const std_msgs::Int8& mspeed){
  Mspeed = mspeed.data;
}

ros::Subscriber<std_msgs::Int8> sub("/motor_speed", &messageCb );
char info[16];

void setup()
{
  AFMS.begin();
  nh.initNode();
}

void loop()
{
  nh.spinOnce();
  setMotorSpeed(Mspeed, Mspeed);
  forwardMotion();
  itoa(Mspeed, info, 10);
  nh.loginfo(info);
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
