#include <SoftwareSerial.h>

#include <Wire.h>
#include <Adafruit_MotorShield.h>
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *rightMotor = AFMS.getMotor(2); // Right Motor
Adafruit_DCMotor *leftMotor = AFMS.getMotor(1); // Left Motor

int RSpeed = 30;
int LSpeed = 30;
float V = 0;
float w = 0;
float d = 63.7/1000;
boolean newData = false;
// V | w | stop
float cvals[3] = {V, w, 0};

const byte numChars = 32;
char caseData = 'i';
char inputData[numChars];

void setup() {
  AFMS.begin();
  Serial.begin(9600);
  Serial.println("<Arduino is ready>");

}

void loop() {
  recvWithEndMarker();
  parseNewData();

  if (cvals[2] == 1) {
    RSpeed = 0;
    LSpeed = 0;
  }
  else {
    V = cvals[0];
    w = cvals[1];

    float LCalc = V-w*d/2;
    float RCalc = V+w*d/2;
    LSpeed = int((LCalc/0.25)*255);
    RSpeed = int((RCalc/0.25)*255);
  }

  setMotorSpeed(RSpeed, LSpeed);

  if (LSpeed >= 0) {
    leftMotor->run(FORWARD);
  }
  else {
    leftMotor->run(BACKWARD);
  }

  if (RSpeed >= 0) {
    rightMotor->run(FORWARD);
  }
  else {
    rightMotor->run(BACKWARD);
  }
  
  delay(50);

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
    float zoomy = atof(inputData);
    newData = false;

    switch (caseData) {
      case 'i': case 'I':
        Serial.println("Initial case");
        cvals[0] = 0;
        cvals[1] = 0;
        cvals[2] = 1;
        break;
        
      case 'v': case 'V': // read in linear velocity
        Serial.print("Linear velocity:");
        Serial.println(zoomy);
        cvals[0] = zoomy;
        break;
        
      case 'w': case 'W': // read in angular velocity
        Serial.print("Angular velocity:");
        Serial.println(zoomy);
        cvals[1] = zoomy;
        break;
        
      case 'x': case 'X': // stop
        Serial.println("Stopping");
        cvals[2] = 1;
        break;

      case 'g': case 'G':
        Serial.println("GO");
        cvals[2] = 0;
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

void setMotorSpeed(int RMspeed, int LMspeed) {
  rightMotor->setSpeed(abs(RMspeed));
  leftMotor->setSpeed(abs(LMspeed));
}
