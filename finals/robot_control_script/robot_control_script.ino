#include <Encoder.h>
#include <Servo.h>
#include <AutoPID.h>

// MOTOR DEFINITIONS
const static int LEFTDIR = 10; // left motor input 1
const static int LEFTPWM = 12; // left motor input 2
const static int RIGHTDIR = 7; // right motor input 1
const static int RIGHTPWM = 8; // right motor input 2

// WHEEL DEFINITIONS
const static int RADIUS = 40; // RADIUS in mm
const static int CIRCUMFERENCE = RADIUS*2*PI; // wheel CIRCUMFERENCE

// ENCODER DEFINITIONS
const static int NUMPERREV = 2560;

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
double left_val, right_val, left_setpoint, right_setpoint, left_output, right_output;
const static int OUTPUT_MIN = -255;
const static int OUTPUT_MAX = 255;
const static int KP = .12;
const static int KI = .0003;
const static int KD = 0;
// creating left and right control loops
AutoPID leftPid(&left_val, &left_setpoint, &left_output, OUTPUT_MIN, OUTPUT_MAX, KP, KI, KD);
AutoPID rightPid(&right_val, &right_setpoint, &right_output, OUTPUT_MIN, OUTPUT_MAX, KP, KI, KD);



void setup() {
  // sets starting encoder position to zero
  leftEnc.write(0);
  rightEnc.write(0);

  // sets motor pins as outputs
  pinMode(LEFTDIR, OUTPUT);
  pinMode(LEFTPWM, OUTPUT);
  pinMode(RIGHTDIR, OUTPUT);
  pinMode(RIGHTPWM, OUTPUT);
  
  myServo.attach(SERVOPIN); // attach servo to pin
  draw(false); // has pen not drawing by default

  //if temperature is more than 5 mm below or above setpoint, OUTPUT will be set to min or max respectively
  leftPid.setBangBang(5);
  rightPid.setBangBang(5);
  //set PID update interval to 10ms
  leftPid.setTimeStep(10);
  rightPid.setTimeStep(10);
}

void loop() {
  draw(true);
  set_motor(LEFTDIR, LEFTPWM, 200);
  set_motor(RIGHTDIR, RIGHTPWM, 80);
}



void go_to_pos(float lm, float rm) {
  /* 
   * Pass in desired encoder values and motors will go until position is hit
   * 
   * lm: float which contains desired left wheel position
   * rm: float which contains desired right wheel position
   */
  left_setpoint = lm;  // updates setpoint in PID object
  right_setpoint = rm;

  while (true) {
    left_val = read_encoder(leftEnc);
    right_val = read_encoder(rightEnc);
  
    leftPid.run();
    rightPid.run();
  
    set_motor(LEFTDIR, LEFTPWM, left_output);
    set_motor(RIGHTDIR, RIGHTPWM, right_output);

    if (abs(left_output) < 10 && abs(right_output) < 10) {
      break; 
    }
  }
}

float read_encoder(Encoder encoder) { 
  /*
   * Reads encoder value and converts it to position in mm
   * 
   * encoder: Encoder object to read values from
   */
  float value = encoder.read();

  // converts encoder value to mm
  value = CIRCUMFERENCE * value / NUMPERREV / 300;

  return value; // in mm
}

void set_motor(int dir, int pwm, int value) {
  // Turns motor off
  digitalWrite(dir, LOW);
  digitalWrite(pwm, LOW);
  delay(20);

  // Drives motor backwards
  if (value < 0) {
    digitalWrite(dir, LOW);
    analogWrite(pwm, abs(value));
  }
  // Drives motor forward
  else if (value > 0) {
    digitalWrite(dir, HIGH);
    analogWrite(pwm, abs(value));
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
