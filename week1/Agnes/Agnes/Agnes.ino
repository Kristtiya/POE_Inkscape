#include <Wire.h>
#include <Adafruit_MotorShield.h>
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *rightMotor = AFMS.getMotor(2); // Right Motor
Adafruit_DCMotor *leftMotor = AFMS.getMotor(1); // Left Motor

// initialize variable for the motor speed
int Mspeed = 30;

// initialize variable for determining whether to accept new data or not
boolean newData = false;

// Motor Speed | Motor override (stop) | Direction
int cvals[3] = {30, 0, 0};

// Serial Input Work
const byte numChars = 32;
char caseData = 'p';
char inputData[numChars]; // an array to store the received data


void setup() {
  AFMS.begin();
  Serial.begin(9600);
  Serial.println("<Arduino is ready>");
}


void loop() {
  // Read Serial input, if any
  recvWithEndMarker();
  parseNewData();

  // If override, set motor speed to 0
  if (cvals[1] == 1) {
    Mspeed = 0;
  }
  else {
    Mspeed = cvals[0];
  }

  // move the robot
  setMotorSpeed(Mspeed, Mspeed);
  
  // If Direction == 0, go forward. Else backward.
  if (cvals[2] == 0) {
    forwardMotion();
  }
  else {
    backwardMotion();
  }
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

/// read all the data from serial input until a new line appears
void recvWithEndMarker() {
  static byte ndx = 0;
  char endMarker = '\n';
  char rc;
  while (Serial.available() > 0 && newData == false) {
    rc = Serial.read();
    if (rc != endMarker) {
      if (ndx == 0) {
        caseData = rc;
      }
      else {
        inputData[ndx - 1] = rc;
      }
      ndx++;
      if (ndx > numChars) {
        ndx = numChars - 1;
      }
    }
    else {
      inputData[ndx - 1] = '\0'; // terminate the string
      ndx = 0;
      newData = true;
    }
  }
}


/// uses serial connection to change the motor speed and right/left sensor threshold and stop/go
void parseNewData() {
  if (newData == true) {
    int zoomy = atoi(inputData);
    newData = false;

    switch (caseData) {
      case 'm': case 'M': // change motor speed
        Serial.println("Motor speed:");
        Serial.println(zoomy);
        cvals[0] = zoomy;
        break;
      case 'x': case 'X': // change left sensor threshold
        Serial.println("Stop time.");
        cvals[1] = 1;
      case 'g': case 'G': // move the robot
        Serial.println("Gotta go fast");
        cvals[1] = 0;
        break;
      case 'w': case 'W': // Go forward
        Serial.println("Going forward.");
        cvals[2] = 0;
        break;
      case 's': case 'S': // Go backward
        Serial.println("Going backward.");
        cvals[2] = 1;
        break;
      case 'a': case 'A': // Turn left
        Serial.print("Turning left for ");
        Serial.print(zoomy);
        Serial.println(" ms.");
        turnLeft(zoomy);
        break;
      case 'd': case 'D': // Turn right
        Serial.print("Turning right for ");
        Serial.print(zoomy);
        Serial.println(" ms.");
        turnRight(zoomy);
        break;
      default:  // in case of error, stop the robot, set motor speed to 0
        Serial.println("Something screwed up.");
        cvals[0] = 0;
        cvals[1] = 1;
        cvals[2] = 0;
        break;
    }
  }
}


/// print the IR sensor output values for testing
void printSensorOutput(int RSvalue, int LSvalue) {
  Serial.print("Right Value: ");
  Serial.print(RSvalue);
  Serial.print (" | Left Value: ");
  Serial.println(LSvalue);
}
