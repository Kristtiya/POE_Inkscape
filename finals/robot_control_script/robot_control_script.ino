#include <Encoder.h>
#include <Servo.h>
#include <AutoPID.h>

// MOTOR DEFINITIONS
const static int LEFTDIR = 10; // left motor input 1
const static int LEFTPWM = 11; // left motor input 2
const static int RIGHTDIR = 7; // right motor input 1
const static int RIGHTPWM = 6; // right motor input 2

// WHEEL DEFINITIONS
const static int RADIUS = 40; // RADIUS in mm
const static int CIRCUMFERENCE = RADIUS*2*PI; // wheel CIRCUMFERENCE

// ENCODER DEFINITIONS
const static int NUMPERREV = 1416;

const static int LEFTE1 = 3; // left encoder 1
const static int LEFTE2 = 5; // left encoder 2
Encoder leftEnc(LEFTE1, LEFTE2);

const static int RIGHTE1 = 2; // right encoder 1
const static int RIGHTE2 = 4; // right encoder 2
Encoder rightEnc(RIGHTE1,RIGHTE2);

// SERVO DEFINITION
// myServo.write(val); val goes from 0-180
const static int SERVOPIN = 9; // servo pin on arduino
const static int UP = 180; // servo value to have pen drawing
const static int DOWN = 120; // servo value to have pen not drawing
Servo myServo;

// PID DEFINITIONS
//pid settings and gains
double left_setpoint, right_setpoint, left_output, right_output;
double left_val, right_val;
const static int OUTPUT_MIN = -255;
const static int OUTPUT_MAX = 255;
const static int KP = 40;
const static int KI = 0;
const static int KD = 0;
// creating left and right control loops
AutoPID leftPid(&left_val, &left_setpoint, &left_output, OUTPUT_MIN, OUTPUT_MAX, KP, KI, KD);
AutoPID rightPid(&right_val, &right_setpoint, &right_output, OUTPUT_MIN, OUTPUT_MAX, KP, KI, KD);

// WiFi Connection Params
const unsigned int writeInterval = 50; // write interval (in ms)
float tcVals[5] = {-1,-1,0,0,0};
const byte numChars = 32;
char inputData[numChars];
static byte ndx = 0;
char endMarker = '>';
char rc;
int numCount = 0;
unsigned long timeout = millis();

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
  leftPid.setBangBang(30);
  rightPid.setBangBang(30);
  //set PID update interval to 10ms
  leftPid.setTimeStep(10);
  rightPid.setTimeStep(10);
  Serial.begin(115200);
}


void loop() {
//  set_motor(LEFTDIR, LEFTPWM, 0);
//  set_motor(RIGHTDIR, RIGHTPWM, 0);
//  draw(true);
//  delay(5000);
//  draw(false);
//  delay(1000);
//  Serial.print("Left Encoder: ");
//  Serial.print(read_encoder(leftEnc));
//  Serial.print("  ||  Right Encoder: ");
//  Serial.println(read_encoder(rightEnc));
//  go_to_pos(80, 80);
//  left_setpoint = 1;
//  right_setpoint = 1;
//  
//  left_val = read_encoder(leftEnc);
//  right_val = read_encoder(rightEnc);
//    
//  leftPid.run();
//  rightPid.run();
//
////  Serial.print("left: ");
////  Serial.print(left_output);
////  Serial.print("  val: ");
////  Serial.print(left_val);
////  Serial.print("  ||  right: ");
////  Serial.print(right_output);
////  Serial.print("  val: ");
////  Serial.println(right_val);
//  set_motor(LEFTDIR, LEFTPWM, left_output);
//  set_motor(RIGHTDIR, RIGHTPWM, right_output);

  recvNums();
  go_to_pos();
}

bool go_to_pos() {
  /* 
   * Pass in desired encoder values and motors will go until position is hit
   * 
   * lm: float which contains desired left wheel position
   * rm: float which contains desired right wheel position
   */
  left_setpoint = tcVals[0];  // updates setpoint in PID object
  right_setpoint = tcVals[1];

  if (tcVals[2] == 0) {
    draw(true);
  }
  else {
    draw(false);
  }

  left_val = read_encoder(leftEnc);
  right_val = read_encoder(rightEnc);

  leftPid.run();
  rightPid.run();

  set_motor(LEFTDIR, LEFTPWM, left_output);
  set_motor(RIGHTDIR, RIGHTPWM, right_output);

  if (abs(left_output-tcVals[0]) < 5 && abs(right_output-tcVals[1]) < 5) {
    set_motor(LEFTDIR, LEFTPWM, 0);
    set_motor(RIGHTDIR, RIGHTPWM, 0);
    return true;
  }
  return false;
}

int read_encoder(Encoder encoder) { 
  /*
   * Reads encoder value and converts it to position in mm
   * 
   * encoder: Encoder object to read values from
   */
  float value = encoder.read();

  // converts encoder value to mm
  value = CIRCUMFERENCE * value / NUMPERREV;

  return int(value); // in mm
}

void set_motor(int dir, int pwm, int value) {
  // Turns motor off
  digitalWrite(dir, LOW);
  digitalWrite(pwm, LOW);
  delay(25);

  // Drives motor backwards
  if (value < 0) {
    digitalWrite(dir, LOW);
    analogWrite(pwm, 255-abs(value));
  }
  // Drives motor forward
  else if (value > 0) {
    digitalWrite(dir, HIGH);
    analogWrite(pwm, 255-abs(value));
  }
  
}

void draw(bool yes) {
  // pen gets pushed DOWN
  int val = UP;
  if (yes) {
    val = DOWN;
  }
  myServo.write(val);
}

void recvNums() {
  timeout = millis();
  while (Serial.available() == 0) {
    if (millis() - timeout > 1000) {
      return;
    }
  }
  while (Serial.available()) {
    rc = Serial.read();
    if (rc != endMarker) {
      if (rc == ',') {
        tcVals[numCount] = atof(inputData);
        ndx = -1;
        numCount++;
      }
      else {
        inputData[ndx] = rc;
      }
      ndx++;
      if (ndx >= numChars) {
        ndx = numChars - 1;
      }
    }
    else {
      tcVals[numCount] = atof(inputData);
      inputData[ndx] = '\0'; // terminate the string
      ndx = 0;
      numCount = 0;

      pushData();
      return;
    }
  }

  tcVals[numCount] = atof(inputData);
  inputData[ndx] = '\0'; // terminate the string
  ndx = 0;
  numCount = 0;

  pushData();
  return;
}

void pushData() {
  unsigned long tt = millis();
  Serial.print(left_val);
  Serial.print(",");
  Serial.print(right_val);
  Serial.print(",");
  Serial.print(tt);
  Serial.print(">");
}

