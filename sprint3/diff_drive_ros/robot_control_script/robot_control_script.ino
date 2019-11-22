#include <Encoder.h>
#include <Servo.h>
#include <AutoPID.h>

// MOTOR DEFINITIONS
const static int LEFTM1 = 10; // left motor input 1
const static int LEFTM2 = 11; // left motor input 2
const static int RIGHTM1 = 12; // right motor input 1
const static int RIGHTM2 = 13; // right motor input 2
Servo lm1;
Servo lm2;
Servo rm1;
Servo rm2;

// WHEEL DEFINITIONS
const static int RADIUS = 40; // RADIUS in mm
const static int CIRCUMFERENCE = RADIUS*2*PI; // wheel CIRCUMFERENCE

// ENCODER DEFINITIONS
const static int NUMPERREV = 2568;

const static int LEFTE1 = 2; // left encoder 1
const static int LEFTE2 = 4; // left encoder 2
Encoder leftEnc(LEFTE1, LEFTE2);

const static int RIGHTE1 = 3; // right encoder 1
const static int RIGHTE2 = 8; // right encoder 2
Encoder rightEnc(RIGHTE1,RIGHTE2);

// SERVO DEFINITION
// myServo.write(val); val goes from 0-180
const static int SERVOPIN = 7; // servo pin on arduino
const static int UP = 180; // servo value to have pen drawing
const static int DOWN = 130; // servo value to have pen not drawing
Servo myServo;

// PID DEFINITIONS
//pid settings and gains
double left_val, right_val, left_setpoint, right_setpoint, left_ouput, right_output;
const static int OUTPUT_MIN = -255;
const static int OUTPUT_MAX = 255;
const static int KP = .12;
const static int KI = .0003;
const static int KD = 0;
// creating left and right control loops
AutoPID left_pid(&left_val, &left_setpoint, &left_output, OUTPUT_MIN, OUTPUT_MAX, KP, KI, KD);
AutoPID right_pid(&right_val, &right_setpoint, &right_output, OUTPUT_MIN, OUTPUT_MAX, KP, KI, KD);

void setup() {
  // sets starting encoder position to zero
  left_enc.write(0);
  right_enc.write(0);

  // sets motor pins as outputs
//  pinMode(LEFTM1, OUTPUT);
//  pinMode(LEFTM2, OUTPUT);
//  pinMode(RIGHTM1, OUTPUT);
//  pinMode(RIGHTM2, OUTPUT);
  lm1.attach(LEFTM1);
  lm2.attach(LEFTM2);
  rm1.attach(RIGHTM1);
  rm2.attach(RIGHTM2);
  
  myServo.attach(SERVOPIN); // attach servo to pin
  draw(false); // has pen not drawing by default

  //if temperature is more than 5 mm below or above setpoint, OUTPUT will be set to min or max respectively
  myPID.setBangBang(5);
  //set PID update interval to 10ms
  myPID.setTimeStep(10);
}

void loop() {
  draw(true);
  set_motor(lm1, lm2, 20);
  set_motor(rm1, rm2, 20);
}

void go_to_pos(float lm, float rm) {
  left_setpoint = lm;
  right_setpoint = rm;

  while (true) {
    left_val = read_encoder(left_enc);
    right_val = read_encoder(right_enc);
  
    myPID.run();
  
    set_motor(lm1, lm2, left_output);
    set_motor(rm1, rm2, right_output);

    if (abs(left_output) < 10 && abs(right_output) < 10) {
      break; 
    }
  }
}

void read_encoder(Encoder encoder) { 
  float value = encoder.read();

  // converts encoder value to mm
  value = CIRCUMFERENCE * value / NUMPERREV / 300;

  return value; // in mm
}

void set_motor(Servo m1, Servo m2, int value) {
  // Turns motor off
  if (value == 0) {
    m1.write(value);
    m2.write(value);
  }
  // Drives motor forward
  else if (value > 0) {
    m1.write(abs(value));
    m2.write(0);
  }
  // Drives motor backwards
  else {
    m1.write(abs(value));
    m2.write(0);
  }
}

void draw(bool yes) {
  // pen gets pushed DOWN
  if (yes){
    myServo.write(DOWN);
  }
  // pen stops drawing
  else {
    myServo.write(UP);
  }
}
