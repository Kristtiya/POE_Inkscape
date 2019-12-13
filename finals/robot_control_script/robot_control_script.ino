#include <Encoder.h>
#include <Servo.h>
#include <AutoPID.h>

#define sgn(x) ((x) < 0 ? -1 : ((x) > 0 ? 1 : 0))

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
const static int SERVOPIN = 8; // servo pin on arduino
const static int UP = 180; // servo value to have pen drawing
const static int DOWN = 120; // servo value to have pen not drawing
Servo myServo;

// PID DEFINITIONS
//pid settings and gains
double left_setpoint, right_setpoint, left_output, right_output;
double left_val, right_val;
const static int OUTPUT_MIN = -250;
const static int OUTPUT_MAX = 250;
const static int LP = 4;
const static int LI = 0.001;
const static int LD = 60;
const static int RP = 4;
const static int RI = 0.001;
const static int RD = 80;
// creating left and right control loops
AutoPID leftPid(&left_val, &left_setpoint, &left_output, OUTPUT_MIN, OUTPUT_MAX, LP, LI, LD);
AutoPID rightPid(&right_val, &right_setpoint, &right_output, OUTPUT_MIN, OUTPUT_MAX, RP, RI, RD);
//int signL = 0;
//int signR = 0;
const static int threshold = 10;

// WiFi Connection Params
const unsigned int writeInterval = 50; // write interval (in ms)
float tcVals[5] = {0,0,1.0,0,0};
const byte numChars = 32;
char inputData[numChars];
static byte ndx = 0;
char endMarker = '>';
char rc;
int numCount = 0;
unsigned long timeout = millis();
bool move_made = true;;

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
  draw(true); // has pen not drawing by default

  //set PID update interval to 10ms
  leftPid.setTimeStep(1);
  rightPid.setTimeStep(1);
  Serial.begin(115200);

  delay(30000);
}


void loop() {
  draw(true);
  draw_square();
  
}

bool go_to_pos() {
  /* 
   * Pass in desired encoder values and motors will go until position is hit
   */
  left_setpoint = tcVals[0];  // updates setpoint in PID object
  right_setpoint = tcVals[1];

  // sees if it should be drawing
  if (tcVals[2] == 0) {
    draw(true);
  }
  else {
    draw(true);
  }

  // updates encoder values
  left_val = read_encoder(leftEnc);
  right_val = read_encoder(rightEnc);

  // runs PID loop to get motor values
  leftPid.run();
  rightPid.run();

  // sets motors to speed
  set_motor(LEFTDIR, LEFTPWM, left_output);
  set_motor(RIGHTDIR, RIGHTPWM, right_output);

  // returns after it is within 2 cm of desired position
  if (leftPid.atSetPoint(20) && leftPid.atSetPoint(20)) {
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
    analogWrite(pwm, abs(value));
  }
  // Drives motor forward
  else if (value > 0) {
    digitalWrite(dir, HIGH);
    analogWrite(pwm, 255-abs(value));
  }
  
}

void draw_square(){
  tcVals[2] = 0;
  for (int i = 0; i<10000; i++){
    tcVals[0] += 200;
    tcVals[1] += 200;
    while (true){
      if (go_to_pos()) {
        break;
      }
    }

    tcVals[0] -= 80;
    tcVals[1] += 90;
    while (true){
      if (go_to_pos()) {
        break;
      }
    }
  }
  tcVals[2] = 1;
}

void draw(bool yes) {
  /*
   * Controls drawing servo
   * 
   * yes: bool
   */
   
  // pen gets pushed DOWN
  int val = UP;
  if (yes) {
    val = DOWN;
  }
  myServo.write(val);
}

/* Receives and sends values to the ESP8266
 *   And therefore the computer
 * Code is motivated from online forum named Serial Input Basics
 * Read it here: https://forum.arduino.cc/index.php?topic=396450.0
 * It will explain the basics of how this code functions.
 * Additional part of code:
 *   While receiving data from ESP, it also parses out numbers
 *   It delimits successive numbers via commas
 *   The parsed out values are stored in an array
 */
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

      pushData(); // After receiving data, send response
      return;
    }
  }

  tcVals[numCount] = atof(inputData);
  inputData[ndx] = '\0'; // terminate the string
  ndx = 0;
  numCount = 0;

  pushData(); // After receiving data, send response
  return;
}

void pushData() {
  // Send necessary values to ESP
  // Format: comma-separated numbers in a string
  unsigned long tt = millis();
  Serial.print(left_val);
  Serial.print(",");
  Serial.print(right_val);
  Serial.print(",");
  Serial.print(tt);
  Serial.print(">");
}
