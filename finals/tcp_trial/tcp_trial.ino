/*
  Connect Arduino WiFi to AskSensors
 * Description:  This sketch connects the Arduino to AskSensors IoT Platform (https://asksensors.com) 
 * Uses an ESP8266 WiFi.
 *  Author: https://asksensors.com, 2018
 *  github: https://github.com/asksensors
 */
 

//#include <SoftwareSerial.h>

// serial config
//#define     RX    10
//#define     TX    11
//SoftwareSerial ESP(RX,TX); 

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
  Serial.begin(115200);
//  ESP.begin(57600);
}

void loop() {
  recvNums();
  delay(writeInterval);   // delay
}

//void echoString() {
//  Serial.println("Echo function called");
//  endMarker = '\n';
//  while (ESP.available()) {
//    Serial.println("Should be reading");
//    rc = ESP.read();
//    if (rc != endMarker) {
//      inputData[ndx] = rc;
//      ndx++;
//      if (ndx > numChars) {
//        ndx = numChars - 1;
//      }
//    }
//    else {
//      inputData[ndx - 1] = '\0'; // terminate the string
//      ndx = 0;
//    }
//  }
//  Serial.println(inputData);
//}

/// read all the data from serial input until a new line appears
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

      Serial.print(tcVals[0]);
      Serial.print(", ");
      Serial.print(tcVals[1]);
      Serial.print(">");
      return;
    }
  }

  tcVals[numCount] = atof(inputData);
  inputData[ndx] = '\0'; // terminate the string
  ndx = 0;
  numCount = 0;

  Serial.print(tcVals[0]);
  Serial.print(", ");
  Serial.print(tcVals[1]);
  Serial.print(">");
  return;
}


