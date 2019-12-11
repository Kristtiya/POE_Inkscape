#include <Wire.h>
#include <Adafruit_MotorShield.h>
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *rightMotor = AFMS.getMotor(2); // Right Motor
Adafruit_DCMotor *leftMotor = AFMS.getMotor(1); // Left Motor

/// initialize variable for the motor speed
int Mspeed = 30;
int x;
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

  // set motor speeds with the parameter
  uint8_t i;

  if (Serial.available() > 0) {
    // here '1' (the character) is important as 1 is the number
    // and '1' equals 0x31 (ASCII)
    x = Serial.read();
    if (x == 0) {   //When no signal, no run
      rightMotor->setSpeed(0);
      leftMotor->setSpeed(0);
      
   } else if (x == B01) {           //Right Wheel Backward
      rightMotor->setSpeed(0);
      leftMotor->setSpeed(Mspeed);
      rightMotor->run(FORWARD);
      leftMotor->run(FORWARD);
      
    } else if (x == B10) {          //Right Wheel Forward
      rightMotor->setSpeed(Mspeed);
      leftMotor->setSpeed(0);
      rightMotor->run(FORWARD);
      leftMotor->run(FORWARD);
      
    } else if (x == B11) {            //Left Wheel Back
      rightMotor->setSpeed(0);
      leftMotor->setSpeed(Mspeed);
      leftMotor->run(BACKWARD);
      
    } else if (x == B100) {             //Right Wheel back
      rightMotor->setSpeed(Mspeed);
      leftMotor->setSpeed(0);
      rightMotor->run(BACKWARD);

    }else if (x == B101) { 
      Mspeed = Mspeed + 1;
    
    }else if (x == B110) { 
      Mspeed = Mspeed -1 ;
    
    } else {   //If no cases fulfilled, no run
      rightMotor->setSpeed(0);
      leftMotor->setSpeed(0);
    }
   
  }
}
